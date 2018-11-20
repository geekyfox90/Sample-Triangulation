cmake_minimum_required(VERSION 3.7.2)

##################################################
project("SolARTriangulationSample")
set (SOURCES main.cpp)
##################################################

# various macros
include("$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/solarmacros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()

# define the list of files to copy to build directory
set(FILES_TO_COPY
	$ENV{BCOMDEVROOT}/thirdParties/opencv/${OPENCV_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}opencv_world${OPENCVVERSIONSUFFIX}.${LIBEXTENSION}

	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_filesystem.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_system.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_timer.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_log.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_chrono.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_thread.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_date_time.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/thirdParties/boost/${BOOST_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}boost_fiber.${LIBEXTENSION}

	$ENV{BCOMDEVROOT}/bcomBuild/SolARModuleOpenCV/${SOLARMODULEOPENCV_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}SolARModuleOpenCV.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/${SOLARFRAMEWORK_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}SolARFramework.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/bcomBuild/SolARModuleTools/${SOLARMODULETOOLS_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}SolARModuleTools.${LIBEXTENSION}

	$ENV{BCOMDEVROOT}/thirdParties/xpcf/${XPCF_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}xpcf.${LIBEXTENSION}	

	$ENV{BCOMDEVROOT}/thirdParties/freeglut/${FREEGLUT_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}${LIBGLUT}glut.${LIBEXTENSION}

	$ENV{BCOMDEVROOT}/bcomBuild/SolARModuleOpenCV/${SOLARMODULEOPENCV_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}SolARModuleOpenCV.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/${SOLARFRAMEWORK_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}SolARFramework.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/bcomBuild/SolARModuleTools/${SOLARMODULETOOLS_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}SolARModuleTools.${LIBEXTENSION}
	$ENV{BCOMDEVROOT}/bcomBuild/SolARModuleOpenGL/${SOLARMODULEOPENGL_VERSION}/lib/x86_64/shared/${BUILDCONFIG}/${LIBPREFIX}SolARModuleOpenGL.${LIBEXTENSION}

	${CMAKE_CURRENT_SOURCE_DIR}/conf_Triangulation.xml
	${CMAKE_CURRENT_SOURCE_DIR}/conf_Triangulation_nf.xml
	${CMAKE_CURRENT_SOURCE_DIR}/camera_calibration.yml
	${CMAKE_CURRENT_SOURCE_DIR}/Image1.png
	${CMAKE_CURRENT_SOURCE_DIR}/Image2.png

	)
# define targets (library, install and uninstall)
defineTargets("executable" "${FILES_TO_COPY}")

