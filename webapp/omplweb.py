#!/usr/bin/env python

import os
from os.path import dirname, abspath, join, basename
import json
import sys
import tempfile
import webbrowser
from inspect import isclass

# The ConfigParser module has been renamed to configparser in Python 3.0
try:
    import ConfigParser
except:
    import configparser as ConfigParser

# Constants
show_results = False
ompl_app_root = dirname(dirname(abspath(__file__)))
ompl_web_root = join(dirname(dirname(abspath(__file__))), "webapp")
ompl_sessions_dir = join(ompl_app_root, 'webapp/static/sessions')
problem_files = join(ompl_app_root, 'webapp/static/problem_files')


try:
    import ompl
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import control as oc
    from ompl import app as oa
    from ompl.util import getLogLevel, setLogLevel, getOutputHandler, LogLevel, OutputHandler
    sys.path.insert(0, join(ompl_app_root, 'ompl/scripts'))
    from ompl_benchmark_statistics import readBenchmarkLog
except:
    sys.path.insert(0, join(ompl_app_root, 'ompl/py-bindings'))
    import ompl
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import control as oc
    from ompl import app as oa
    from ompl.util import getLogLevel, setLogLevel, getOutputHandler, LogLevel, OutputHandler
    sys.path.insert(0, join(ompl_app_root, 'ompl/scripts'))
    from ompl_benchmark_statistics import readBenchmarkLog

import flask
from flask import Flask
from werkzeug import secure_filename

from celery import Celery
from celery.result import AsyncResult

# Configure Flask and Celery
app = flask.Flask(__name__)
app.config.from_object(__name__)
celery = Celery(app.name, broker='amqp://', backend='rpc://')


class Logger(OutputHandler):
    """
    Handles logging at different levels, extends OutputHandlerSTD.
    """

    def __init__(self):
        self.out = getOutputHandler()
        setLogLevel(LogLevel.LOG_DEBUG)


    def log(self, text, level, filename, line):
        if level >= getLogLevel():
            self.out.log(text, level, filename, line)


oh = Logger()
oh.log("Log level is set to: %d" % getLogLevel(), LogLevel.LOG_INFO, "omplweb.py", 75)
ompl.initializePlannerLists()


########## OMPL ##########

def allowed_file(filename):
    """
    Checks that the parameter is a .dae file.
    """

    if '.' in filename and filename.rsplit('.', 1)[1] == 'dae':
        # Extract the file extension and return true if dae
        return True

    return False


def parse_cfg(cfg_path):
    """
    Parses the configuration file for pre-defined problems and returns a
    settings dictionary
    """

    if (sys.version_info > (3, 0)):
        config = ConfigParser.ConfigParser(strict = False)
    else:
        config = ConfigParser.ConfigParser()

    config.readfp(open(cfg_path, 'r'))

    return config._sections['problem']


def save_cfg_file(name, session_id, text):
    """
    Saves a .cfg file intended for benchmarking
    """
    file_loc = join(ompl_sessions_dir, session_id, name);
    f = open(file_loc + ".cfg", 'w')
    f.write(text)
    f.close()

    return file_loc


def format_solution(path, solved):
    """
    Formats the either the solution, or a failure message for delivery to the
    client.
    """

    solution = {}

    if solved:
        solution['solved'] = 'true'
        # Format the path
        path_matrix = path.printAsMatrix().strip().split('\n')

        # A list of n states, where path_list[0] is the start state and path_list[n]
        # is the goal state, and path_list[i] are the intermediary states.
        # Each state is also a list: [x, y, z,
        path_list = []

        for line in path_matrix:
            # print "\t " + line
            path_list.append(line.strip().split(" "))

        # print path_list
        solution['path'] = path_list;

        solution['pathAsMatrix'] = path.printAsMatrix()
    else:
        solution['solved'] = 'false'

    return solution


def get_offset(env_mesh, robot_mesh):
    ompl_setup = oa.SE3RigidBodyPlanning()
    ompl_setup.setEnvironmentMesh(str(env_mesh))
    ompl_setup.setRobotMesh(str(robot_mesh))

    # full state
    start = ompl_setup.getDefaultStartState()
    # just the first geometric component
    start = ompl_setup.getGeometricComponentState(start,0)
    # extract x,y,z coords
    # print start
    offset = {}
    offset['x'] = start[0]
    offset['y'] = start[1]
    offset['z'] = start[2]
    return offset


def setup(problem):
    oh.log("Robot type is: %s" % str(problem["robot.type"]), LogLevel.LOG_INFO, "omplweb.py", 172)

    ompl_setup = eval("oa.%s()" % problem["robot.type"])
    problem["is3D"] = isinstance(ompl_setup.getGeometricComponentStateSpace(), ob.SE3StateSpace)

    ompl_setup.setEnvironmentMesh(str(problem['env_loc']))
    ompl_setup.setRobotMesh(str(problem['robot_loc']))

    if problem["is3D"]:
        # Set the dimensions of the bounding box
        bounds = ob.RealVectorBounds(3)

        bounds.low[0] = float(problem['volume.min.x'])
        bounds.low[1] = float(problem['volume.min.y'])
        bounds.low[2] = float(problem['volume.min.z'])

        bounds.high[0] = float(problem['volume.max.x'])
        bounds.high[1] = float(problem['volume.max.y'])
        bounds.high[2] = float(problem['volume.max.z'])

        ompl_setup.getGeometricComponentStateSpace().setBounds(bounds)

        space = ob.SE3StateSpace()
        # Set the start state
        start = ob.State(space)
        start().setXYZ(float(problem['start.x']), float(problem['start.y']), float(problem['start.z']))

        # Set the start rotation
        start().rotation().x = float(problem['start.q.x'])
        start().rotation().y = float(problem['start.q.y'])
        start().rotation().z = float(problem['start.q.z'])
        start().rotation().w = float(problem['start.q.w'])

        # Set the goal state
        goal = ob.State(space)
        goal().setXYZ(float(problem['goal.x']), float(problem['goal.y']), float(problem['goal.z']))

        # Set the goal rotation
        goal().rotation().x = float(problem['goal.q.x'])
        goal().rotation().y = float(problem['goal.q.y'])
        goal().rotation().z = float(problem['goal.q.z'])
        goal().rotation().w = float(problem['goal.q.w'])

        ompl_setup.setStartAndGoalStates(start, goal)
    else:
        print("Non 3D robot type.")

        # Set the dimensions of the bounding box
        bounds = ob.RealVectorBounds(2)

        bounds.low[0] = float(problem['volume.min.x'])
        bounds.low[1] = float(problem['volume.min.y'])

        bounds.high[0] = float(problem['volume.max.x'])
        bounds.high[1] = float(problem['volume.max.y'])

        ompl_setup.getGeometricComponentStateSpace().setBounds(bounds)
        space = ob.SE2StateSpace()

        start = ob.State(space)
        start().setX(float(problem['start.x']))
        start().setY(float(problem['start.y']))
        # start().setYaw(self.rot.value() * pi / 180)

        goal = ob.State(space)
        goal().setX(float(problem['goal.x']))
        goal().setY(float(problem['goal.y']))
        # goal().setYaw(self.rot.value() * pi / 180)

        ompl_setup.setStartAndGoalStates(start, goal)

    return ompl_setup


@celery.task()
def solve(problem):
    """
    Given an instance of the Problem class, containing the problem configuration
    data, solves the motion planning problem and returns either the solution
    path or a failure message.
    """

    # Sets up the robot type related information
    ompl_setup = setup(problem);

    # Load the planner
    space_info = ompl_setup.getSpaceInformation()
    planner = eval("%s(space_info)" % problem['planner'])
    ompl_setup.setPlanner(planner)

    # Set the optimization objective
    obj = eval('ob.%s(space_info)' % problem['optimization.objective'])
    cost = ob.Cost(float(problem['cost.threshold']))
    obj.setCostThreshold(cost)
    ompl_setup.setOptimizationObjective(obj)

    # Set each parameter that was configured by the user
    for param in problem['planner_params']:
        planner.params().setParam(str(param), str(problem['planner_params'][param]))

    # Solve the problem
    solution = {}
    solved = ompl_setup.solve(float(problem['solve_time']))

    # Check for validity
    if solved:
        path = ompl_setup.getSolutionPath()
        initialValid = path.check()

        if initialValid:
            try:
                # If initially valid, attempt to simplify
                ompl_setup.simplifySolution()
                # Get the simplified path
                simple_path = ompl_setup.getSolutionPath()
                simplifyValid = simple_path.check()
                if simplifyValid:
                    path = simple_path;

            except:
                oh.log("Error occurred in trying to simplify solution.", LogLevel.LOG_ERROR, "omplweb.py", 284)

            # Interpolate path
            ns = int(100.0 * float(path.length()) / float(ompl_setup.getStateSpace().getMaximumExtent()))
            path.interpolate(ns)
            if len(path.getStates()) != ns:
                oh.log("Interpolation produced " + str(len(path.getStates())) + " states instead of " + str(ns) + " states.", LogLevel.LOG_WARN, "omplweb.py", 256)

            solution = format_solution(path, True)
        else :
            solution = format_solution(path, False)
    else:
        solution = format_solution(None, False)

    solution['name'] = str(problem['name'])
    solution['planner'] = ompl_setup.getPlanner().getName()

    return solution

@celery.task()
def solve_multiple(runs, problem):
    """
    """
    result = {}
    result['multiple'] = "true"
    solutions = []

    for i in range(0, runs):
        oh.log("Solving run number: {}".format(i), LogLevel.LOG_INFO, "omplweb.py", 276)
        solutions.append(solve(problem))

    result['solutions'] = solutions

    return result

@celery.task()
def benchmark(name, session_id, cfg_loc, db_filename, problem_name, robot_loc, env_loc):
    """
    Runs ompl_benchmark on cfg_loc and converts the resulting log file to a
    database with ompl_benchmark_statistics.

    cfg_loc - the location of the .cfg file to be benchmarked
    """

    if problem_name != "custom":
        robot_file = join(ompl_sessions_dir, session_id, basename(robot_loc));
        env_file = join(ompl_sessions_dir, session_id, basename(env_loc));
        if not os.path.isfile(robot_file) and not os.path.isfile(env_file):
            # Copy over the needed mesh files to perform benchmarking, if they don't exist
            os.symlink(join(ompl_web_root, robot_loc), robot_file)
            os.symlink(join(ompl_web_root, env_loc), env_file)

    # Run the benchmark, produces .log file
    os.system("ompl_benchmark " + cfg_loc + ".cfg")

    # Convert .log into database
    dbfile = join(ompl_sessions_dir, session_id, db_filename)
    logfile = []
    logfile.append(join(ompl_sessions_dir, session_id, name + ".log"))
    readBenchmarkLog(dbfile, logfile, "")

    # Open the planner arena page when benchmarking is done
    if show_results:
        url = "http://127.0.0.1:8888/?user=" + session_id + "&job=" + db_filename
        webbrowser.open(url)


########## Flask ##########

# Page Loading
@app.route("/")
def index():
    return flask.redirect(flask.url_for('omplapp'))

@app.route("/omplapp", methods=['GET'])
def omplapp():
    """
    Application starting point, loads everything.
    """
    return flask.render_template("omplweb.html")

@app.route('/omplapp/components/configuration')
def load_configuration():
    return flask.render_template("components/configuration.html")

@app.route('/omplapp/components/benchmarking')
def load_benchmarking():
    return flask.render_template("components/benchmarking.html")

@app.route('/omplapp/components/about')
def load_about():
    return flask.render_template("components/about.html")

@app.route('/omplapp/session')
def create_session():
    """
    Creates a session folder and returns its name
    """
    if not os.path.isdir(ompl_sessions_dir):
        os.mkdir(ompl_sessions_dir)

    session_path = tempfile.mkdtemp(prefix="", dir=ompl_sessions_dir)
    session_name = basename(session_path)

    return session_name
@app.route('/omplapp/preferences')
def load_preferences():
    """
    Reads the webapp's configuration file and sends the preferences to the client.
    """
    if (sys.version_info > (3, 0)):
        config = ConfigParser.ConfigParser(strict = False)
    else:
        config = ConfigParser.ConfigParser()

    config.readfp(open("webapp.cfg", "r"))
    preferences = config._sections["preferences"]

    if preferences["launch_browser_benchmarking_complete"] == "true":
        show_results = True

    return json.dumps(preferences)


# Problem Configuration
@app.route('/omplapp/planners')
def planners():
    return json.dumps(og.planners.plannerMap)

@app.route('/omplapp/offset', methods=["POST"])
def find_offset():
    env_mesh = flask.request.form['env_loc']
    robot_mesh = flask.request.form['robot_loc']

    offset = get_offset(join(ompl_web_root, env_mesh), join(ompl_web_root, robot_mesh))
    return json.dumps(offset)

@app.route('/omplapp/robot_types')
def get_robot_types():
    """
    Finds the available robot types and returns as a list of tuples
    """

    robot_types = {}
    for c in dir(oa):
        if eval('isclass(oa.%s) and issubclass(oa.%s, (oa.AppBaseGeometric,oa.AppBaseControl)) and issubclass(oa.%s, oa.RenderGeometry)' % (c,c,c)):
            name = eval('oa.%s().getName()' % c)
            apptype = eval('oa.%s().getAppType()' % c)
            robot_types[str(c)] = {"name" : str(name), "apptype" : str(apptype)}
    return json.dumps(robot_types)

@app.route('/omplapp/upload', methods=['POST'])
def upload():
    """
    This function is invoked when the client clicks 'Solve' and submits the
    problem configuration data. Problem configuration data is parsed and loaded
    and the problem is solved by an asynchronous celery task. The task ID of
    the task is returned.
    """

    problem = flask.request.get_json(True, False, True)

    runs = int(problem['runs'])
    if runs > 1:
        solve_task = solve_multiple.delay(runs, problem)
        oh.log("Started solving multiple runs with task id: " + solve_task.task_id, LogLevel.LOG_DEBUG, "webapp.py", 354)
        return str(solve_task.task_id)
    else:
        solve_task = solve.delay(problem)
        oh.log("Started solving task with id: " + solve_task.task_id, LogLevel.LOG_DEBUG, "webapp.py", 354)
        return str(solve_task.task_id)

@app.route('/omplapp/poll/<task_id>', methods=['POST'])
def poll(task_id):
    """
    Checks if the task corresponding to the input ID has completed. If the
    task is done solving, the solution is returned.
    """

    result = solve.AsyncResult(task_id)

    if result.ready():
        return json.dumps(result.get()), 200
    else :
        return "Result for task id: " + task_id + " isn't ready yet.", 202

@app.route("/omplapp/upload_models", methods=['POST'])
def upload_models():
    """
    Uploads the user's robot and environment files and saves them to the
    server. The URL to these files are then returned for use by ColladaLoader
    to visualize.
    """

    robot = flask.request.files['robot']
    env = flask.request.files['env']

    session_id = flask.request.form['session_id']
    session_dir = join(ompl_sessions_dir, session_id)

    file_locs = {}

    # Check that the uploaded files are valid
    if robot and env:
        if allowed_file(robot.filename) and allowed_file(env.filename):

            # If valid files, save them to the server
            robot_file = join(session_dir, secure_filename(robot.filename))
            robot.save(robot_file)
            file_locs['robot_loc'] = join("static/sessions", session_id, basename(robot_file))

            env_file = join(session_dir, secure_filename(env.filename))
            env.save(env_file)
            file_locs['env_loc'] = join("static/sessions", session_id, basename(env_file))

        else:
            return "Error: Wrong file format. Robot and environment files must be .dae"
    else:
        return "Error: Didn't upload any files! Please choose both a robot and environment file in the .dae format."

    return json.dumps(file_locs)

@app.route("/omplapp/request_problem", methods=['POST'])
def request_problem():
    """
    Sends the user the user the location of the requested problem's model files
    and the problem configuration settings
    """
    problem_name = flask.request.form['problem_name']
    dimension = flask.request.form['dimension']
    cfg_filename = problem_name + ".cfg"

    cfg_file = join(problem_files, dimension, cfg_filename)
    cfg_data = parse_cfg(cfg_file)

    if (sys.version_info > (3, 0)):
        config = ConfigParser.ConfigParser(strict = False)
    else:
        config = ConfigParser.ConfigParser()
    config.readfp(open(cfg_file, 'r'))

    cfg_data['robot_loc'] = join("static/problem_files", dimension, config.get("problem", "robot"))
    cfg_data['env_loc'] = join("static/problem_files", dimension, config.get("problem", "world"))
    return json.dumps(cfg_data)


# Benchmarking
@app.route('/omplapp/benchmark', methods=['POST'])
def init_benchmark():

    session_id = flask.request.form['session_id']
    session_dir = join(ompl_sessions_dir, session_id)
    cfg = flask.request.form['cfg']
    cfg_name = flask.request.form['filename']
    problem_name = flask.request.form['problem']
    env_loc = flask.request.form["env_loc"]
    robot_loc = flask.request.form["robot_loc"]
    cfg_loc = save_cfg_file(cfg_name, session_id, cfg)

    db_file, db_filepath = tempfile.mkstemp(suffix=".db", prefix="", dir=session_dir)

    # Close the db_file, since we don't need it right now
    os.close(db_file)

    db_filename = basename(db_filepath)

    result = benchmark.delay(cfg_name, session_id, cfg_loc, db_filename, problem_name, env_loc, robot_loc)

    return db_filename


if __name__ == "__main__":
    app.debug = True
    app.run()


