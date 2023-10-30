package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

class PixelIntake(private val opMode: OpMode, private val motorLift: DcMotorEx, private val motorSpin: DcMotorEx) {
    init {
        motorLift.targetPosition = 0
        motorLift.mode = DcMotor.RunMode.RUN_TO_POSITION
        motorSpin.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorSpin.power = 0.0
        motorLift.power = 0.2
//        servoRight
    }

    var active: Boolean = false
        set(status) {
            field = status
            motorSpin.power = if (status) 1.0 else 0.0
        }

    fun lower(): Unit {
        motorLift.targetPosition = 10
    }

    fun raise(): Unit {
        motorLift.targetPosition = 0
    }
}