package org.firstinspires.ftc.teamcode.testing

import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.drive.KMecanumOdoDrive
import org.firstinspires.ftc.teamcode.testing.Hardware

class Robot(startPose: Pose) {
    private val hardware = Hardware(startPose)

    val drive = KMecanumOdoDrive(
        hardware.fl,
        hardware.bl,
        hardware.br,
        hardware.fr,
        hardware.odometry,
        true
    )

//    val arm = Arm(hardware.arm)

    init {
//        arm.motor.setProfileTarget(Arm.initPos)
//        arm.motor.setPositionTarget(Arm.initPos)
    }
}