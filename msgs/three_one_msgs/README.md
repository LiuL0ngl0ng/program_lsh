three_one_msgs
├── CMakeLists.txt
├── msg
│ ├── control.msg  # total control of the car
│   ├── control_speed.msg  # control the speed of the car
│   ├── control_steer.msg  # control the steer of the car
│   ├── control_suspension.msg  # control the suspension of the car
│   ├── rawdata_recv.msg  # udp receive rawdata for the car
│   ├── rawdata_send.msg  # udp send rawdata to the car
│   ├── report_control_data.msg  # feedback of control data from ecu_communication, for comparation and analysis
│   ├── report_cylinder_position.msg  # feedback of cyclinder position
│   ├── report_cylinder_pressure.msg  # feedback of cyclinder pressure
│   ├── report_distance.msg  # feedback of mileage and pulse counter
│   ├── report_give_back.msg  # feedback of control data from ecu
│   ├── report_motion.msg  # feedback speed info of the car
│   ├── report.msg  # total feedback of the car
│   ├── report_torque.msg  # feedback of torque
│   └── report_vehicle_state.msg  # feedback of functions, such as light, ring.
├── package.xml
└── readme.md
