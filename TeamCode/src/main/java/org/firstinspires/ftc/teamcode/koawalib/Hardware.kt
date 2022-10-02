package org.firstinspires.ftc.teamcode.koawalib

import com.asiankoala.koawalib.hardware.motor.MotorFactory
import com.asiankoala.koawalib.hardware.servo.KServo
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import org.firstinspires.ftc.teamcode.koawalib.vision.SleevePipeline
import org.firstinspires.ftc.teamcode.koawalib.vision.Webcam

class Hardware() {
    val fl = MotorFactory("fl")
        .forward
        .brake
        .build()

    val bl = MotorFactory("bl")
        .forward
        .brake
        .build()

    val br = MotorFactory("br")
        .reverse
        .brake
        .build()

    val fr = MotorFactory("fr")
        .reverse
        .brake
        .build()

    val slides = MotorFactory("Slides")
        .reverse
        .brake
        .build()

    val webcam = Webcam("Webcam")

    val arm = KServo("Arm")
    val claw = KServo("Claw").startAt(Claw.openPos)




}