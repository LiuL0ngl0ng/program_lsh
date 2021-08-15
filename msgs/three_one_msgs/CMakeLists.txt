cmake_minimum_required(VERSION 2.8.3)
project(three_one_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
    message_generation
)

add_message_files(
    DIRECTORY
    msg
    FILES
	ControlSpeed.msg
	ControlSteer.msg
	ControlSuspension.msg
	ControlWeapon.msg
	Control.msg
	ReportMotion.msg
	ReportTorque.msg
	ReportDistance.msg
	ReportVehicleState.msg
	ReportCylinderPosition.msg
	ReportCylinderPressure.msg
	ReportGiveBack.msg
	ReportControlData.msg
    Report.msg
	RawdataRecv.msg
	RawdataSend.msg
)

generate_messages()

catkin_package(
    CATKIN_DEPENDS
    message_runtime
)