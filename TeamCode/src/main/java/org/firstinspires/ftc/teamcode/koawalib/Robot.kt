package org.firstinspires.ftc.teamcode.koawalib

import org.firstinspires.ftc.teamcode.koawalib.subsystems.Arm
import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.drive.KMecanumDrive
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Slides
import org.firstinspires.ftc.teamcode.koawalib.vision.SleevePipeline
import org.firstinspires.ftc.teamcode.koawalib.vision.WebcamDevice

class Robot() {
    val hardware = Hardware()

    val drive = KMecanumDrive(
        hardware.fl,
        hardware.bl,
        hardware.br,
        hardware.fr
    )

    val armServo = Arm(hardware.arm)
    val clawServo = Claw(hardware.claw)
    val slidesMotor = Slides(hardware.slides)

    val webcam = WebcamDevice(hardware.webcam, SleevePipeline(0.166, 578.272, 578.272, 402.145, 221.506))

}