package org.firstinspires.ftc.teamcode.drivetrain.messages

import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Time

class DriveCommandMessage(poseVelocity: PoseVelocity2dDual<Time>) {
    var timestamp: Long
    var forwardVelocity: Double
    var forwardAcceleration: Double
    var lateralVelocity: Double
    var lateralAcceleration: Double
    var angularVelocity: Double
    var angularAcceleration: Double

    init {
        this.timestamp = System.nanoTime()
        this.forwardVelocity = poseVelocity.linearVel.x.get(0)
        this.forwardAcceleration = poseVelocity.linearVel.x.get(1)
        this.lateralVelocity = poseVelocity.linearVel.y.get(0)
        this.lateralAcceleration = poseVelocity.linearVel.y.get(1)
        this.angularVelocity = poseVelocity.angVel.get(0)
        this.angularAcceleration = poseVelocity.angVel.get(1)
    }
}
