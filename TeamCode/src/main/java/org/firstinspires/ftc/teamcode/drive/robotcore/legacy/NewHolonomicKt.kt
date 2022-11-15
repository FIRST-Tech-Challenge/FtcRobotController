package org.firstinspires.ftc.teamcode.drive.robotcore.legacy

import com.qualcomm.robotcore.hardware.DcMotor

class NewHolonomicKt(
        private val frontLeftMotor : DcMotor,
        private val frontRightMotor: DcMotor,
        private val backLeftMotor  : DcMotor,
        private val backRightMotor : DcMotor) {

    private fun run(x: Double, y:Double, z:Double) {
        frontLeftMotor.power  =  x + y + z
        backLeftMotor.power   = -x + y + z
        frontRightMotor.power =  x - y + z
        backRightMotor.power  = -x - y + z
    }

    fun runWithoutEncoderXYZ(x: Double, y:Double, z:Double) {
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        run(x, y, z)
    }

    fun runWithoutEncoderPrime(xPrime: Double, yPrime: Double, z: Double) {
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        frontLeftMotor.power = xPrime + z
        backLeftMotor.power = yPrime + z
        backRightMotor.power = -xPrime + z
        frontRightMotor.power = -yPrime + z
    }

    fun setMotorsMode(mode: DcMotor.RunMode) {
        frontLeftMotor.mode  = mode
        frontRightMotor.mode = mode
        backLeftMotor.mode   = mode
        backRightMotor.mode  = mode
    }

}
