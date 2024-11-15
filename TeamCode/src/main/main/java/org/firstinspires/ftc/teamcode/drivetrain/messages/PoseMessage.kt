package org.firstinspires.ftc.teamcode.drivetrain.messages

import com.acmerobotics.roadrunner.Pose2d

class PoseMessage(pose: Pose2d) {
    var timestamp: Long = System.nanoTime()
    var x: Double = pose.position.x
    var y: Double = pose.position.y
    var heading: Double = pose.heading.toDouble()
}

