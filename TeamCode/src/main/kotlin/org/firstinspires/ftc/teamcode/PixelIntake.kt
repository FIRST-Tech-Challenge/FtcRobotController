package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo

class PixelIntake(private val opMode: OpMode, private val servoLeft: Servo, private val servoRight: Servo, val tubeMotor: DcMotorEx) {
    init {
        tubeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        tubeMotor.power = 0.0
    }

    var active: Boolean = false
        set(status) {
            field = status
            tubeMotor.power = if (status) 0.5 else 0.0
        }

    fun lower() {
        servoLeft.position = -0.1
        servoRight.position = 0.1
    }

    fun raise() {
        servoLeft.position = 0.0
        servoRight.position = 0.0
    }
}