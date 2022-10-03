package org.firstinspires.ftc.teamcode.koawalib

import com.asiankoala.koawalib.subsystem.drive.KMecanumDrive
import org.firstinspires.ftc.teamcode.koawalib.vision.SleevePipeline

class Robot() {
    val hardware = Hardware()

    val drive = KMecanumDrive(
        hardware.fl,
        hardware.bl,
        hardware.br,
        hardware.fr
    )

//    val armServo = Arm(hardware.arm)
//    val clawServo = Claw(hardware.claw)
//    val slidesMotor = Slides(hardware.slides)

//    val webcam = WebcamDevice(hardware.webcam, SleevePipeline(0.166, 578.272, 578.272, 402.145, 221.506))

}