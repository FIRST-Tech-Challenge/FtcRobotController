package org.firstinspires.ftc.teamcode.koawalib

import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.drive.KMecanumDrive
import com.asiankoala.koawalib.subsystem.drive.KMecanumOdoDrive
import org.firstinspires.ftc.teamcode.koawalib.vision.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.koawalib.vision.SleevePipeline
import org.firstinspires.ftc.teamcode.koawalib.vision.WebcamDevice

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

//    val armServo = Arm(hardware.arm)
//    val clawServo = Claw(hardware.claw)
//    val slidesMotor = Slides(hardware.slides)

    val webcam = WebcamDevice(hardware.webcam, AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506))

}