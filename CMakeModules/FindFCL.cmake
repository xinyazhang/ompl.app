include(FindPackageHandleStandardArgs)
find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(FCL fcl>=0.3.1)
    if(FCL_LIBRARIES AND NOT FCL_INCLUDE_DIRS)
        set(FCL_INCLUDE_DIRS "/usr/include")
    endif()
else()
    # vcpkg installs cmake config files for fcl
    find_package(FCL CONFIG)
    if(TARGET fcl)
        get_target_property(FCL_INCLUDE_DIRS fcl INTERFACE_INCLUDE_DIRECTORIES)
        get_target_property(FCL_LIBRARIES fcl INTERFACE_LINK_LIBRARIES)
        set(FCL_LIBRARIES "fcl;${FCL_LIBRARIES}")
    endif()
endif()
find_package_handle_standard_args(FCL DEFAULT_MSG FCL_LIBRARIES FCL_INCLUDE_DIRS)
