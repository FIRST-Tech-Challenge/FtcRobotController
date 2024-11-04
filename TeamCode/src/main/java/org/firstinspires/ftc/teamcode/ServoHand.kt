package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Servo

class ServoHand(private val servo: Servo, private val openPosition: Double, private val closePosition: Double) {
    private var isClosed: Boolean = false

    init {
        servo.position = closePosition
    }

    fun grab(close: Boolean) {
        if (close) {
            servo.position = closePosition
        } else {
            servo.position = openPosition
        }

        isClosed = close
    }

    fun toggle() {
        grab(!isClosed)
    }
}