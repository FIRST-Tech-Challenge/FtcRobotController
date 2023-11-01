package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

@Suppress("unused")
class PixelIntake(private val opMode: OpMode, private val motorLift: DcMotorEx, private val motorSpin: DcMotorEx) {
    init {
        motorLift.targetPosition = 0
        motorLift.mode = DcMotor.RunMode.RUN_TO_POSITION
        motorSpin.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorSpin.power = 0.0
        motorLift.power = 0.2
//        servoRight
    }

    // A is off, B is inwards, C is outwards
    var active: Ternary = Ternary.A
        set(status) {
            field = status
            motorSpin.power = when (status) {
                Ternary.A -> 0.0
                Ternary.B -> 1.0
                Ternary.C -> -1.0
            }
        }
//    var isRaised: Boolean = false;

    fun lower() {
        motorLift.targetPosition = 10
    }

    fun raise() {
        motorLift.targetPosition = 0
    }
}