package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx


class Motor(val dcMotorEx: DcMotorEx) {

    operator fun invoke() : DcMotorEx {
        return dcMotorEx
    }

    fun reset() {
        dcMotorEx.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        dcMotorEx.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun runToPosition(target: Int, power: Double) {
        dcMotorEx.targetPosition = target
        dcMotorEx.mode = DcMotor.RunMode.RUN_TO_POSITION
        dcMotorEx.power = power

        while (dcMotorEx.isBusy) {
            //it does stuff here
        }

        reset()
    }

}