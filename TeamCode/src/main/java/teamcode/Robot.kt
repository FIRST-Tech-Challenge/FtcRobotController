package org.firstinspires.ftc.teamcode.koawalib

import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.drive.KMecanumOdoDrive
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Arm
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lift
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lights
import org.firstinspires.ftc.teamcode.koawalib.vision.SleevePipeline

class Robot(startPose: Pose) {
    val hardware = Hardware(startPose)

    val drive = KMecanumOdoDrive(
        hardware.fl,
        hardware.bl,
        hardware.br,
        hardware.fr,
        hardware.odometry,
        true
    )

    val arm = Arm(hardware.armMotor)
    val claw = Claw(hardware.clawServo)
    val lift = Lift(hardware.liftLeadMotor, hardware.liftSecondMotor)
//    val lightsDevice = Lights(hardware.lights)

    init {
        arm.setPos(-67.7)
        lift.setPos(0.0)
    }
}