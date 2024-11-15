package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain

interface Robot {
    val drivetrain: Drivetrain
    var position: Pose2d
    var velocity: PoseVelocity2d // = PoseVelocity2d(Vector2d(0.0, 0.0), 0.0),
    val positionHistory: MutableList<Pose2d> //= LinkedList<Pose2d>(),
    val velocityHistory: MutableList<PoseVelocity2d> //= LinkedList<PoseVelocity2d>(),
}
