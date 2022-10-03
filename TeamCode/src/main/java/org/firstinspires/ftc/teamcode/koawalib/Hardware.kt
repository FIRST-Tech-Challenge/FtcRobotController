package org.firstinspires.ftc.teamcode.koawalib

import com.asiankoala.koawalib.hardware.motor.MotorFactory
import com.asiankoala.koawalib.hardware.servo.KServo

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

//    val slides = MotorFactory("Slides")
//        .reverse
//        .brake
//        .build()


//    val arm = KServo("Arm")
//    val claw = KServo("Claw").startAt(Claw.openPos)




}