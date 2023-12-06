package org.firstinspires.ftc.teamcode.botmodule

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

/**
 * Intake controls
 */
@Suppress("unused")
class Intake(opMode: OpMode, private val motorLift: DcMotorEx?, private val motorSpin: DcMotorEx?) : BotModule(opMode) {
    init {
        motorLift?.targetPosition = 0
        motorLift?.mode = DcMotor.RunMode.RUN_TO_POSITION
        motorSpin?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorSpin?.power = 0.0
        motorLift?.power = 0.2
//        servoRight
    }

    // 0.0 is off, 1.0 is inwards, -1.0 is outwards
    var active: Double = 0.0
        set(status) {
            if (motorSpin == null) field = 0.0
            else {
                field = status
                motorSpin.power = if (field > 1.0) 1.0 else if (field < -1.0) -1.0 else field
            }
        }
    var raised: Boolean = false
        private set

    fun lower() {
        motorLift?.targetPosition = 10
        raised = false
    }

    fun raise() {
        motorLift?.targetPosition = 0
        raised = true
    }
}