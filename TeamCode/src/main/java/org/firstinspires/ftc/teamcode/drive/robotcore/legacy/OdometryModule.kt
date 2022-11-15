package org.firstinspires.ftc.teamcode.drive.robotcore.legacy

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class OdometryModule(private val encoder: DcMotor,
                     private val reverse: Boolean = false) {

    init {
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private var currentPositionOffset = 0

    fun resetHWCounter() {
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun resetSWCounter() { currentPositionOffset = encoder.currentPosition }

    fun getDistanceRaw() = (encoder.currentPosition - currentPositionOffset).times(if (reverse) -1 else 1)

    fun getDistanceNormalized(unit: DistanceUnit = DistanceUnit.INCH) = unit.fromMm(((encoder.currentPosition - currentPositionOffset).toDouble()/ TICKS_PER_REVOLUTION) * Math.PI * WHEEL_DIAMETER_mm).times(if (reverse) -1.0 else 1.0)

    companion object {
        const val TICKS_PER_REVOLUTION = 8192
        const val WHEEL_DIAMETER_mm = 38
    }
}