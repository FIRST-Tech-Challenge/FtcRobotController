package org.firstinspires.ftc.teamcode.koawalib

import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.drive.KMecanumOdoDrive
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Arm
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lift
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lights
import org.firstinspires.ftc.teamcode.koawalib.vision.SleevePipeline
import org.firstinspires.ftc.teamcode.koawalib.vision.Webcam

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

    val arm = Arm(hardware.armMotor)
    val claw = Claw(hardware.clawServo, hardware.distanceSensor)
    val lift = Lift(hardware.liftLeadMotor, hardware.liftSecondMotor)
    val webcam = Webcam(hardware.webcam, SleevePipeline(0.166, 578.272, 578.272, 402.145, 221.506))
    val lightsDevice = Lights(hardware.lights)
}