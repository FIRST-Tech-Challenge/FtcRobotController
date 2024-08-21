package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor

class SwerveModule (topMotor: DcMotor, bottomMotor: DcMotor, x: Double, y: Double) {
    var top: DcMotor = topMotor
    var bottom: DcMotor = bottomMotor

    fun resetEncoders(useEncoder: Boolean = true) {
        top.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        bottom.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        if (useEncoder) {
            top.mode = DcMotor.RunMode.RUN_USING_ENCODER
            bottom.mode = DcMotor.RunMode.RUN_USING_ENCODER
        } else {
            top.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            bottom.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }
}