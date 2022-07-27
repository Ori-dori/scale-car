# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;pluginlib;nodelet;dynamic_reconfigure;ackermann_msgs;rospy;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lackermann_cmd_mux_nodelet;-lyaml-cpp".split(';') if "-lackermann_cmd_mux_nodelet;-lyaml-cpp" != "" else []
PROJECT_NAME = "ackermann_cmd_mux"
PROJECT_SPACE_DIR = "/home/wego/catkin_ws/install"
PROJECT_VERSION = "0.7.0"
