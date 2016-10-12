# Copyright: (C) 2010 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_device(BasicCartesianControl
                    TYPE teo::BasicCartesianControl
                    INCLUDE BasicCartesianControl.hpp
                    WRAPPER CartesianControlServer)
yarp_install(FILES BasicCartesianControl.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

IF (NOT SKIP_BasicCartesianControl)

SET(CMAKE_MODULE_PATH ${TEO_MODULE_PATH} ${CMAKE_MODULE_PATH})
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${TEO_INCLUDE_DIRS})

yarp_add_plugin(BasicCartesianControl BasicCartesianControl.hpp DeviceDriverImpl.cpp ICartesianControlImpl.cpp RateThreadImpl.cpp)

TARGET_LINK_LIBRARIES(BasicCartesianControl ${YARP_LIBRARIES} ${TEO_LIBRARIES})

#install(TARGETS BasicCartesianControl DESTINATION lib)
yarp_install(TARGETS BasicCartesianControl
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

# Exporting dependencies for TEOConfig.cmake quite manually for now... 
#set(TEO_INCLUDE_DIRS ${TEO_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")
#set(TEO_LIBRARIES ${TEO_LIBRARIES} TeoYarp BasicCartesianControl CACHE INTERNAL "appended libraries")

ENDIF (NOT SKIP_BasicCartesianControl)

