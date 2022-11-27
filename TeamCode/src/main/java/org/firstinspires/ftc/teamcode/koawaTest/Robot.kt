package org.firstinspires.ftc.teamcode.koawaTest

import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.drive.KMecanumOdoDrive
import org.firstinspires.ftc.teamcode.koawaTest.subsystems.Arm
import org.firstinspires.ftc.teamcode.koawaTest.subsystems.Lift

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

    val arm = Arm(hardware.arm)
    val lift = Lift(hardware.liftMotor, hardware.liftSecondMotor)
}