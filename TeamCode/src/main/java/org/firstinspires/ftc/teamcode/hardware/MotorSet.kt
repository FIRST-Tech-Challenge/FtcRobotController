package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor

data class MotorSet(
    val frontLeft: DcMotor,
    val frontRight: DcMotor,
    val backLeft: DcMotor,
    val backRight: DcMotor,
) {
    fun setAll(power: Number) {
        val powerD = power.toDouble()
        frontLeft.power = powerD
        frontRight.power = powerD
        backLeft.power = powerD
        backRight.power = powerD
    }

    fun set(frontLeft: Number, frontRight: Number, backLeft: Number, backRight: Number) {
        this.frontLeft.power = frontLeft.toDouble()
        this.frontRight.power = frontRight.toDouble()
        this.backLeft.power = backLeft.toDouble()
        this.backRight.power = backRight.toDouble()
    }
}
