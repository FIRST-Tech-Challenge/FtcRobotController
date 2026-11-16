package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.hardware.RobotPose
import kotlin.math.PI

class PoseSet {

    var RIGHT_HIVE = true

    val CORNER_START = RobotPose(62.75, 62.75, PI)
    val OTHER_FLOWER_PRESCORE = RobotPose(51.3, 20.4, 0.0)
    val OTHER_FLOWER_INTAKE = RobotPose(59.0, 20.4, 0.0)
    val SCORE_LEFT = RobotPose(57.3, 38.4, 0.0)
    val SCORE_RIGHT = RobotPose(-57.3, 38.4, PI)
    val DRIVER_FLOWER_PRESCORE = RobotPose(-21.4, 54.3, PI/2)
    val DRIVER_FLOWER_INTAKE = RobotPose(-21.4, 62.0, PI/2)
    val PARK_RIGHT = RobotPose(22.7,64.1,PI/2)
    val PARK_LEFT = RobotPose(49.3, 63.6, PI/2)
    val PREGARDEN = RobotPose(-53.3, 65.5, PI)
    val GARDEN = RobotPose(-62.0, 65.5, PI)
    val HIVE_WAYPOINT = RobotPose(0.0, 48.0, PI)
}