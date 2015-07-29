/* Primary objects */
var problem;
var solution;
var visualization;
var benchmark;

/* Global Variables */
var pollingInterval;
var robotAnimationInterval;
var MAX_UPLOAD_SIZE = 50000000; // In bytes, sets limit to 50MB


// Define the Problem class
var Problem = function () {
	// Problem constructor
	this.config = {};

	// Initialize all problem values to 0 or empty
	this.config["name"] = "";
	this.config["robot"] = "";
	this.config["world"] = "";
	this.config["start.x"] = null;
	this.config["start.y"] = null;
	this.config["start.z"] = null;
	this.config["start.q.w"] = null;
	this.config["start.q.x"] = null;
	this.config["start.q.y"] = null;
	this.config["start.q.z"] = null;
	this.config["goal.x"] = null;
	this.config["goal.y"] = null;
	this.config["goal.z"] = null;
	this.config["goal.q.w"] = null;
	this.config["goal.q.x"] = null;
	this.config["goal.q.y"] = null;
	this.config["goal.q.z"] = null;
	this.config["volume.min.x"] = null;
	this.config["volume.min.y"] = null;
	this.config["volume.min.z"] = null;
	this.config["volume.max.x"] = null;
	this.config["volume.max.y"] = null;
	this.config["volume.max.z"] = null;

	this.config["optimization.objective"] = "";
	this.config["cost.threshold"] = null;

	this.config["solve_time"] = 10;
	this.config["runs"] = 1;

	// The URI of the robot and env models on the server, for use by ColladaLoader
	this.config["robot_loc"] = "";
	this.config["env_loc"] = "";

}

Problem.prototype.update = function() {
	// Read all of the input files and update the internal values
	this.config["name"] = $("[name='name']").val();
	this.config["start.x"] = $("[name='start.x']").val();
	this.config["start.y"] = $("[name='start.y']").val();
	this.config["start.z"] = $("[name='start.z']").val();
	this.config["start.q.w"] = start_robot.quaternion.w;
	this.config["start.q.x"] = start_robot.quaternion.x;
	this.config["start.q.y"] = start_robot.quaternion.y;
	this.config["start.q.z"] = start_robot.quaternion.z;
	this.config["goal.x"] = $("[name='goal.x']").val();
	this.config["goal.y"] = $("[name='goal.y']").val();
	this.config["goal.z"] = $("[name='goal.z']").val();
	this.config["goal.q.w"] = goal_robot.quaternion.w;
	this.config["goal.q.x"] = goal_robot.quaternion.x;
	this.config["goal.q.y"] = goal_robot.quaternion.y;
	this.config["goal.q.z"] = goal_robot.quaternion.z;
	this.config["volume.min.x"] = $("[name='volume.min.x']").val();
	this.config["volume.min.y"] = $("[name='volume.min.y']").val();
	this.config["volume.min.z"] = $("[name='volume.min.z']").val();
	this.config["volume.max.x"] = $("[name='volume.max.x']").val();
	this.config["volume.max.y"] = $("[name='volume.max.y']").val();
	this.config["volume.max.z"] = $("[name='volume.max.z']").val();

	this.config["planner"]= $("[name='planners']").val();
	this.config["optimization.objective"] = $("[name='optimization.objective']").val();
	this.config["cost.threshold"] = $("[name='cost.threshold']").val();

	this.config["solve_time"] = $("[name='solve_time']").val();
	this.config["runs"] = $("[name='runs']").val();

	// Get the params for the selected planner
	var paramData = {};
	$('.planner_param').each(function () {
		paramData[$(this).attr('name')] = $(this).val();
	});
	this.config["planner_params"] = paramData;
};

Problem.prototype.get = function(field) {
	return this.config[field]
};

Problem.prototype.asJSON = function() {
	return JSON.stringify(this.config);
};

Problem.prototype.isValid = function() {
	this.update();
	for (var item in this.config) {
		if (this.config[item] == null || this.config[item] === "") {
			console.log(item, this.config[item]);
			return false;
		}
	}
	return true;
};

/**
 * Gathers and formats problem data and submits the problem to the server for solving.
 * On successful solve, saves solution data and loads solution visualization.
 *
 * @param None
 * @return None
 */
Problem.prototype.solve = function() {
	// Ensure that we have the latest data from the user
	problem.update();

	if (problem.isValid()) {
		// Bring up the loading screen
		$.blockUI({
			css: {
				border: 'none',
				padding: '30px',
				backgroundColor: '#000',
				opacity: '0.7',
				color: '#fff',
			}
		});


		// Send the request
		$.ajax({
			url: "omplapp/upload",
			type: "POST",
			data: problem.asJSON(),
			success: function(data){
				var taskID = String(data);
				console.log("Server successfully recieved solve request. Given task ID: " + taskID);
				solution.poll(taskID);
			},
			error: function(data) {
				$.unblockUI();
				showAlert("configuration", "danger", "Server responded with an error. Check the problem configuration and try again.");

				console.log('Solve failed, server responded with an error.', data);
			},
			cache: false,
			contentType: 'application/json',
			processData: false
		});
	} else {
		// Invalid fields have been highlighted by 'validateField()'.
		showAlert("configuration", "warning", "Please enter values for the indicated fields.");
	}
};

Problem.prototype.hasMultipleRuns = function() {
	if (self.config["runs"] > 1) {
		return true;
	}
	return false;
};

/**
 * Formats configuration fields into a .cfg text file
 *
 * @param None
 * @return {string} All the configuration inforamtion in text.
 */
Problem.prototype.getConfigText = function() {
	if (problem.isValid()) {
		var startQ = start_robot.quaternion;
		var goalQ = goal_robot.quaternion;

		var cfg = "";
		cfg += "[problem]\n";

		cfg += "name = " + $("[name='name']").val() + "\n";

		if ($('#problems').val() == "custom"){
			cfg += "robot = " + $("input[name='robot']")[0].files[0].name + "\n";
			cfg += "world = " + $("input[name='env']")[0].files[0].name + "\n";
		} else {
			cfg += "robot = " + $("#problems").val() + "_robot.dae\n";
			cfg += "world = " + $("#problems").val() + "_env.dae\n";
		}

		cfg += "start.x = " + $("[name='start.x']").val() + "\n";
		cfg += "start.y = " + $("[name='start.y']").val() + "\n";
		cfg += "start.z = " + $("[name='start.z']").val() + "\n";

		cfg += "start.axis.x = " + startQ.x + "\n";
		cfg += "start.axis.y = " + startQ.y + "\n";
		cfg += "start.axis.z = " + startQ.z + "\n";
		cfg += "start.theta = " + startQ.w + "\n";

		cfg += "goal.x = " + $("[name='goal.x']").val() + "\n";
		cfg += "goal.y = " + $("[name='goal.y']").val() + "\n";
		cfg += "goal.z = " + $("[name='goal.z']").val() + "\n";

		cfg += "goal.axis.x = " + goalQ.x + "\n";
		cfg += "goal.axis.y = " + goalQ.y + "\n";
		cfg += "goal.axis.z = " + goalQ.z + "\n";
		cfg += "goal.theta = " + goalQ.w + "\n";

		cfg += "volume.min.x = " + $("[name='volume.min.x']").val() + "\n";
		cfg += "volume.min.y = " + $("[name='volume.min.y']").val() + "\n";
		cfg += "volume.min.z = " + $("[name='volume.min.z']").val() + "\n";
		cfg += "volume.max.x = " + $("[name='volume.max.x']").val() + "\n";
		cfg += "volume.max.y = " + $("[name='volume.max.y']").val() + "\n";
		cfg += "volume.max.z = " + $("[name='volume.max.z']").val() + "\n";

		cfg += "\n";
		cfg += "[benchmark]\n";
		cfg += "time_limit = " + $("[name='time_limit']").val() + "\n";
		// Set arbitrary, large mem limit
		cfg += "mem_limit = " + "10000\n"
		cfg += "run_count = " + $("[name='run_count']").val() + "\n";

		cfg += "\n";
		cfg += "[planner]\n";
		cfg += benchmark.getBenchmarkingPlanners();

		return cfg;
	} else {
		showAlert("configuration", "warning", "Please enter values for the indicated fields.");
		return null;
	}
}

Problem.prototype.parseConfigFile = function() {
	var cfgFile = $("#cfg-file")[0].files[0];

	if (cfgFile != null) {
		var reader = new FileReader();
		reader.readAsText(cfgFile);
		reader.onload = function () {
			var cfgData = {};
			// Separate into lines
			var cfgLines = reader.result.split("\n");
			for (var i=0; i < cfgLines.length; i++) {
				// Remove all extra spacing
				var line = cfgLines[i].replace(/\s+/g, '');
				if(line == ""){
					continue;
				} else {
					if(line[0] != "[") {
						// Split into (key, value) pairs
						var items = line.split("=");
						if (items[1] == null){
							throw "Invalid configuration on line containing: " + items[0];
						} else {
							cfgData[items[0]]= items[1];
						}
					} else {
						// This config line isn't used
						// console.log("Ignored: ", cfgLines[i]);
					}
				}
			}
			problem.loadConfigFile(cfgData);
		}
	} else {
		showAlert("configuration", "danger", "Error parsing the configuration file, try again");
	}

}

Problem.prototype.loadConfigFile = function(data) {

	console.log(data);
	// Convert the rotation to degrees around each axis
	var startQ = axisAngleToQuaternion(data['start.axis.x'],
		data['start.axis.y'], data['start.axis.z'], data['start.theta']);
	var startRot = quaternionToAxisDegrees(startQ);

	var goalQ = axisAngleToQuaternion(data['goal.axis.x'],
		data['goal.axis.y'], data['goal.axis.z'], data['goal.theta']);
	var goalRot = quaternionToAxisDegrees(goalQ);

	// Update the input fields with the loaded data
	$("[name='name']").val(data['name']);
	$("[name='start.x']").val(data['start.x']);
	$("[name='start.y']").val(data['start.y']);
	$("[name='start.z']").val(data['start.z']);
	$("[name='start.deg.x']").val(startRot.x);
	$("[name='start.deg.y']").val(startRot.y);
	$("[name='start.deg.z']").val(startRot.z);
	$("[name='goal.x']").val(data['goal.x']);
	$("[name='goal.y']").val(data['goal.y']);
	$("[name='goal.z']").val(data['goal.z']);
	$("[name='goal.deg.x']").val(goalRot.x);
	$("[name='goal.deg.y']").val(goalRot.y);
	$("[name='goal.deg.z']").val(goalRot.z);
	$("[name='volume.min.x']").val(data['volume.min.x']);
	$("[name='volume.min.y']").val(data['volume.min.y']);
	$("[name='volume.min.z']").val(data['volume.min.z']);
	$("[name='volume.max.x']").val(data['volume.max.x']);
	$("[name='volume.max.y']").val(data['volume.max.y']);
	$("[name='volume.max.z']").val(data['volume.max.z']);

	// Benchmarking
	$("[name='time_limit']").val(data['time_limit']);
	$("[name='mem_limit']").val(data['mem_limit']);
	$("[name='run_count']").val(data['run_count']);

	problem.config["robot"] = data["robot"];
	problem.config["world"] = data["world"];

	setTimeout(function() {
		visualization.updatePose();
		visualization.updateBounds();
	}, 100);
}

/**
 * Gets the config data and prompts the user to download it.
 *
 * @param None
 * @return None
 */
Problem.prototype.downloadConfig = function(field) {
	var cfg = this.getConfigText();
	if (cfg != null) {
		var blob = new Blob([cfg], {type: "octet/stream"});
		var cfgName = $("[name='name']").val() + ".cfg";
		downloadFile(blob, cfgName);
	}
};


// Define the Solution class
var Solution = function() {
	// Parse the solution data data
}

/**
 * Polls the server at an interval to check for problem solution. Continues
 * polling until a solution has been found or an error has been returned.
 *
 * @param {string} taskID The ID of the celery task which is solving the problem.
 * @return None
 */
Solution.prototype.poll = function(taskID) {
	var completed = false;
	var pollURL = '/omplapp/poll/' + taskID;

	pollingInterval = window.setInterval(function() {

		$.ajax({
			url: pollURL,
			type: 'POST',
			data: taskID,
			success: function (data, textStatus, jqXHR) {
				if (jqXHR.status == 200) {
					// Stop polling
					clearInterval(pollingInterval);
					solution.store(data);
					solution.visualize();
				} else {
					console.log(data, textStatus, jqXHR);
				}
			},
			error: function (jqXHR, textStatus, errorThrown) {
				$.unblockUI();
				html = "<pre>Server responded with an error. Check the problem configuration and try again.</pre>";
				$('#results').html(html);

				console.log('Solve failed, server responded with an error.', errorThrown);
			}
		});

	}, 2000);
};

Solution.prototype.store = function(solutionData) {
	this.data = JSON.parse(solutionData);
};

Solution.prototype.clear = function() {
	this.data = null;
	clearInterval(robotAnimationInterval);
	visualization.clearSolution();
	$('#pathButtons').addClass('hidden');
}

/**
 * Parses solution JSON from server and displays solution data.
 *
 * @param {string} data The solution data from the server as a JSON string
 * @return None
 */
Solution.prototype.visualize = function() {
	// Hide the bounding box
	$('#showBoundingBox').prop('checked') == false;
	bbox.visible = false;

	// Clear the old solution visualization, if it existed
	visualization.clearSolution();

	if (this.data.multiple === "true") {
		var numSolved = 0;

		// Path animation options are not shown for multiple runs
		$('#pathButtons').addClass('hidden');

		$.each(this.data.solutions, function(index, run) {
			if (run.solved === "true") {
				visualization.drawSolutionPath(run.path);
				numSolved += 1;
			}
		});

		var msg = "Solutions found for " + numSolved + " of " + this.data.solutions.length + " runs.";
		showAlert("configuration", "success", msg);

	} else {
		if (this.data.solved == "true") {
			// Draw the solution path
			visualization.visualizeSolution(this.data);

			visualization.animationSpeed = 1000 - $('#animationSpeed').val();

			$('#pathButtons').removeClass('hidden');

			showAlert("configuration", "success", "Solution found!");
		} else {
			showAlert("configuration", "info", "No solution found. Try solving again.");
		}
	}

	$.unblockUI();
};

/**
 * If a solution has been found, allows the user to download the path.
 *
 * @param None
 * @return None
 */
Solution.prototype.downloadSolutionPath = function() {
	if (this.data.pathAsMatrix != null) {
		var blob = new Blob([this.data.pathAsMatrix], {type: "octet/stream"});
		var pathName = this.data.name + "_path.txt";

		downloadFile(blob, pathName);
	} else {
		showAlert("configuration", "warning", "There is no valid solution path to download.");
	}
};


$(document).ready(function() {
	// Load the configuration page by default
	problem = new Problem();
	solution = new Solution();
	visualization = new Visualization();
	benchmark = new Benchmark();

	initialize();
	$('#configuration-page').click();
});


/**
 * Loads the components of the configuration page and sets up listeners
 * that make the page interactive.
 *
 * @param None
 * @return None
 */
function initialize() {
	$("#configuration").load("omplapp/components/configuration", function () {

		visualization.initialize();
		benchmark.initialize();

		// Retrieve the planners:
		getPlannerData();

		// If this is a new session, get the session id
		if (!sessionStorage.getItem("session_id")){
			getSessionID();
		}

		// When user picks a planner, load the planner params
		$("#planners").change(function() {
			planner_name = $("#planners").val();
			loadPlannerParams(planner_name);
		});

		// Load config data when .cfg file is selected
		$("#cfg-file").change(function (){
			problem.parseConfigFile();
		});

		// Show upload buttons if user selects 'Custom' problem
		$("#problems").change(function() {
			visualization.clearScene();
			if ($("#problems").val() == 'custom'){
				$("#customProblem").collapse('show');
				loadPlannerParams("ompl.geometric.KPIECE1");
			} else {
				$("#customProblem").collapse('hide');

				// Retrieve config data for this problem
				loadRemoteProblem($("#problems").val());

				// Set KPIECE1 as the default planner
				loadPlannerParams("ompl.geometric.KPIECE1");
			}
		});

		// Open the problem config tab
		$('#problem-tab').click()

		// Refresh the viz if pose fields are changed
		$('.pose').change(function () {
			visualization.updatePose();
		});

		// Refresh the viz if the bounds are changed
		$('.bounds').change(function () {
			visualization.updateBounds();
		});

		// Select Visualization Theme
		$('#vizTheme').change(function() {
			var color = $('#vizTheme').val();
			if (color == "light") {
				renderer.setClearColor(0xfafafa);
			} else {
				renderer.setClearColor(0x1a1a1a);
			}
		})

		// Toggle display of bounding box
		$('#showBoundingBox').change(function() {
			if ($('#showBoundingBox').prop('checked') == true) {
				bbox.visible = true;
			} else {
				bbox.visible = false;
			}
		});

		// Toggle display of axis helper
		$('#showAxisHelper').change(function() {
			if ($('#showAxisHelper').prop('checked') == true) {
				axisHelper.visible = true;
			} else {
				axisHelper.visible = false;
			}
		});

		// Adjust animation speed
		$('#animationSpeed').change(function() {
			visualization.animationSpeed = 1000 - $("#animationSpeed").val();
			$('#animateToggleBtn').click();
			$('#animateToggleBtn').click();
		});

		// Load the about page
		$("#about").load("omplapp/components/about");
	});
}


/**
 * Retrieves planners from the server and loads up the available planners on both
 * the configure problem page and benchmarking page
 *
 * @param None
 * @return None
 */
function getPlannerData() {
	$.get( "omplapp/planners", function( data ) {
		problem.availablePlanners = JSON.parse(data);

		$.each(problem.availablePlanners, function(fullName, data){
			var shortName = fullName.split(".")[2];

			// Configure problem page planners
			$('#planners').append($("<option></option>").attr("value", fullName).text(shortName));

			// Benchmarking page available planners
			$('#addingPlanners').append(
				$('<li></li>').append(
					$('<a></li>')
						.attr("class", "dropdown-link")
						.text(shortName)
						.on("click", function() {
							benchmark.addPlanner(fullName);
						})
				)
			);

		});

		// Set the default planner
		loadPlannerParams("ompl.geometric.KPIECE1");
		$('#planners').val("ompl.geometric.KPIECE1");

	});
}


/**
 * Gets the unique identifier for this session from the server and stores
 * it globally. This ID accompanies all future requests to the server
 * to ensure that the necessary files are available.
 *
 * @param None
 * @return None
 */
function getSessionID(){
	$.ajax({
		url: 'omplapp/session',
		type: 'GET',
		success: function (data, textStatus, jqXHR) {
			console.log("Got session id: " + data);
			sessionStorage.setItem("session_id", data);
		},
		error: function (jqXHR, textStatus, errorThrown) {
			console.log("Error getting session id. Please reload the page before continuing.");
			console.log(jqXHR, textStatus, errorThrown);
		},
	});
}


/**
 * Given that the planners have been retrieved from the server, creates the
 * parameter fields for a specific planner and adds them to the page.
 *
 * @param {string} planner_name The planner to setup parameters for.
 * @return None
 */
function loadPlannerParams(planner_name) {
	if (problem.availablePlanners != null) {
		var plannerConfigHTML = "";
		plannerConfigHTML += "<form name='param_form'><table class='table'><caption>";
		plannerConfigHTML += planner_name.split(".")[2];
		plannerConfigHTML += " Options</caption><tbody>";
		params = problem.availablePlanners[planner_name]
		for (var key in params) {
			if (params.hasOwnProperty(key)) {
				plannerConfigHTML += "<tr><td>";
				plannerConfigHTML += params[key][0];
				plannerConfigHTML += "</td><td><input type='text' name='" + key + "' class='planner_param form-control input-sm' value='" + params[key][3] + "'></td></tr>";
			}
		}
		plannerConfigHTML += "</tbody></table></form>"
		$("#plannerPane").html(plannerConfigHTML);
	} else {
		showAlert("configuration", "danger", "Planners are not loaded yet. Please refresh the page and try again.");
	}
}


/**
 * Loads a pre-defined problem from the server by drawing the models and
 * filling in configuration information.
 *
 * @param {string} problem_name The name of problem to load.
 * @return None
 */
function loadRemoteProblem(problemName) {
	var form = {'problem_name' : problemName};

	// Retrieve problem configuration:
	$.ajax({
		url: "omplapp/request_problem",
		type: 'POST',
		data: form,
		success: function (data, textStatus, jqXHR) {
			var data = JSON.parse(data);
			env_loc = data['env_loc'];
			robot_loc = data['robot_loc'];

			// Load the robot and env models
			visualization.drawModels(data['env_loc'], data['robot_loc']);

			var startQ = axisAngleToQuaternion(data['start.axis.x'],
				data['start.axis.y'], data['start.axis.z'], data['start.theta']);
			var startRot = quaternionToAxisDegrees(startQ);

			var goalQ = axisAngleToQuaternion(data['goal.axis.x'],
				data['goal.axis.y'], data['goal.axis.z'], data['goal.theta']);
			var goalRot = quaternionToAxisDegrees(goalQ);

			// Load the data
			$("[name='name']").val(data['name']);
			$("[name='start.x']").val(data['start.x']);
			$("[name='start.y']").val(data['start.y']);
			$("[name='start.z']").val(data['start.z']);
			$("[name='start.deg.x']").val(startRot.x);
			$("[name='start.deg.y']").val(startRot.y);
			$("[name='start.deg.z']").val(startRot.z);
			$("[name='goal.x']").val(data['goal.x']);
			$("[name='goal.y']").val(data['goal.y']);
			$("[name='goal.z']").val(data['goal.z']);
			$("[name='goal.deg.x']").val(goalRot.x);
			$("[name='goal.deg.y']").val(goalRot.y);
			$("[name='goal.deg.z']").val(goalRot.z);
			$("[name='volume.min.x']").val(data['volume.min.x']);
			$("[name='volume.min.y']").val(data['volume.min.y']);
			$("[name='volume.min.z']").val(data['volume.min.z']);
			$("[name='volume.max.x']").val(data['volume.max.x']);
			$("[name='volume.max.y']").val(data['volume.max.y']);
			$("[name='volume.max.z']").val(data['volume.max.z']);

			// Benchmarking
			$("[name='time_limit']").val(data['time_limit']);
			$("[name='mem_limit']").val(data['mem_limit']);
			$("[name='run_count']").val(data['run_count']);

			problem.config["robot"] = data["robot"];
			problem.config["world"] = data["world"];
			problem.config["robot_loc"] = data["robot_loc"];
			problem.config["env_loc"] = data["env_loc"];

		},
		error: function (jqXHR, textStatus, errorThrown) {
			console.log("Error requesting problem.");
			console.log(jqXHR, textStatus, errorThrown);
		}
	});
}


/**
 * Uploads the user's models to the server and then draws them to the scene.
 *
 * @param None
 * @return None
 */
function uploadModels() {
	// Read the input fields
	var formData = new FormData($('form')[0]); //TODO: Improve this, more robust selection
	formData.append('session_id', sessionStorage.getItem("session_id"));

	var valid = validateModels();

	if (valid) {
		// Send the request
		$.ajax({
			url: "omplapp/upload_models",
			type: "POST",
			data: formData,
			success: function(data){
				data = JSON.parse(data);
				problem.config["env_loc"]= data['env_loc'];
				problem.config["robot_loc"] = data['robot_loc'];
				visualization.drawModels(data['env_loc'], data['robot_loc']);

			},
			error: function(data) {
				console.log(data);

				showAlert("configuration", "danger", "Unable to upload files.");
			},
			cache: false,
			contentType: false,
			processData: false
		});
	}
}


/**
 * Validates the user selected environment and robot files.
 *
 * @param None
 * @return {Boolean} A boolean indicating the validity of the files.
 */
function validateModels() {
	env_file = $('#env_path')[0].files[0];
	robot_file = $('#robot_path')[0].files[0];

	if (env_file != null && robot_file != null) {
		if (env_file.name.indexOf(".dae") > 0 && robot_file.name.indexOf(".dae") > 0) {
			if (env_file.size < MAX_UPLOAD_SIZE && robot_file.size < MAX_UPLOAD_SIZE) {
				return true;
			} else {
				var max_size = MAX_UPLOAD_SIZE / 1000000;
				var msg = "Robot and environment files must be smaller than " + max_size + " MB each."
				showAlert("configuration", "warning", msg);
			}
		} else {
			showAlert("configuration", "warning", "Robot and environment files must be in the .dae format.");
		}
	} else {
		showAlert("configuration", "warning", "Please select both robot and environment files in the .dae format.");
	}

	return false;
}


