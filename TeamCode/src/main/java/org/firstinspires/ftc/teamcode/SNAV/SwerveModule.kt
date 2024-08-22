package org.firstinspires.ftc.teamcode.SNAV

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.SNAV.Util.Angle
import kotlin.math.PI

class SwerveModule(val top: DcMotor, val bottom: DcMotor, val distanceFromCenter: Double, motorGearRatio: Double, motorGearTeeth: Int, largeGearTeeth: Int, wheelGearTeeth: Int, val ticksPerRevolution: Double, val motorRPM: Int, val wheelDiameter: Double, var maximizeSpeeds: Boolean = false) {
    val totalGearRatio: Double = largeGearTeeth / motorGearTeeth.toDouble()

    val wheelRPM: Double = motorRPM * wheelGearTeeth.toDouble()
    val maxSpeed: Double = wheelRPM * ((PI * wheelDiameter) / 60) // In/s

    var targetAngle: Angle = Angle(0.0)
    val targetAngleMOE: Double = 10.0

    fun setTranslationalSpeed(translationalSpeed: Double = 0.0) {
        // Translations
        top.power = speedToPower(translationalSpeed)
        bottom.power = -speedToPower(translationalSpeed)
    }

    fun updateAngle() {
        if (getAngle().getDistance(targetAngle) > targetAngleMOE)
        {
            top.power += 0.5
            bottom.power += -0.5
        }
    }

    private fun angleToTicks(angle: Double): Int {
        return ((totalGearRatio * angle) / 360.0 * ticksPerRevolution).toInt()
    }

    private fun ticksToAngle(ticks: Int): Double {
        return ((360 * (ticks/ticksPerRevolution))/totalGearRatio)
    }

    private fun speedToPower(speed: Double): Double {
        return speed/maxSpeed
    }

    // Get the discrepancy between the two encoders
    fun getAngle(): Angle {
        return Angle((top.currentPosition - bottom.currentPosition) / ticksPerRevolution.toDouble())
    }

    fun setMode(mode: DcMotor.RunMode) {
        top.mode = mode
        bottom.mode = mode
    }

    fun resetEncoders() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        setMode(DcMotor.RunMode.RUN_USING_ENCODER)
    }
}