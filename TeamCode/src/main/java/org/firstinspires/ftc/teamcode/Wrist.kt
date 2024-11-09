package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.sign

class Wrist(private val lServo: Servo, private val rServo: Servo) {
    var lServoTarget: Double = 0.0
    var rServoTarget: Double = 0.0

    fun twist(delta: Double) {
        lServoTarget = lServo.position + delta
        rServoTarget = rServo.position - delta

        updatePositions()
    }

    fun turn(delta: Double) {
        lServoTarget = lServo.position + sign(delta) * delta
        rServoTarget = rServo.position + sign(delta) * delta

        updatePositions()
    }

    private fun updatePositions() {
        lServo.position = lServoTarget
        rServo.position = rServoTarget
    }

    fun getTurn(): Double {
        return (lServo.position + rServo.position) / 2
    }

    fun getTwist(): Double {
        return abs(lServo.position - rServo.position)
    }
}

class WristCont(private val lServo: CRServo, private val rServo: CRServo) {
    fun twist(delta: Double) {
        lServo.power = delta
        rServo.power = -delta
    }

    fun turn(delta: Double) {
        lServo.power = sign(delta) * delta
        rServo.power = sign(delta) * delta
    }
}