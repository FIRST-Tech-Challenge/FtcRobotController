package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.HardwareMap
import ftc.rouge.blacksmith.util.clamp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcodekt.util.DataSupplier

@Config
object LiftConfig {
    const val P = 0.0011
    const val I = 0.2
    const val D = 0.0001
    const val F = 0.00001

    const val ZERO = 0
    const val LOW = 1091
    const val MID = 1771
    const val HIGH = 2471

    const val MANUAL_ADJUSTMENT_MULT = 50
}

/**
 * This class represents a lift that can be moved to different heights.
 *
 * @param hwMap a [HardwareMap] object that contains information about the robot's hardware
 * @param voltageScaler a [VoltageScaler] object that helps correct for voltage changes
 * @author KG
 */
class Lift(hwMap: HardwareMap, private val voltageScaler: VoltageScaler) {
    private val liftMotor: Motor
    private val liftPID: PIDFController

    private var liftHeight = 0

    /**
     * The height of the lift. This property is clamped between [LiftConfig.ZERO] and
     * [LiftConfig.HIGH]. Only should really be used for manual control.
     */
    var height: Int
        get() = liftHeight
        set(height) {
            liftHeight = height.clamp(LiftConfig.ZERO, LiftConfig.HIGH).toInt()
        }

    init {
        liftMotor = Motor(hwMap, DeviceNames.LIFT_MOTOR, Motor.GoBILDA.RPM_435)

        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        liftMotor.setRunMode(Motor.RunMode.VelocityControl)
        liftMotor.resetEncoder()

        liftPID = PIDFController(
            LiftConfig.P, LiftConfig.I,
            LiftConfig.D, LiftConfig.F
        )
    }

    /**
     * Move the lift to the zero height. Used for intaking or depositing on ground junctions/terminals.
     */
    fun goToZero() {
        liftHeight = LiftConfig.ZERO
    }

    /**
     * Move the lift to the low height. Used for low poles.
     */
    fun goToLow() {
        liftHeight = LiftConfig.LOW
    }

    /**
     * Move the lift to the mid height. Used for mid poles.
     */
    fun goToMid() {
        liftHeight = LiftConfig.MID
    }

    /**
     * Move the lift to the high height. Used for high poles.
     */
    fun goToHigh() {
        liftHeight = LiftConfig.HIGH
    }

    /**
     * Update the lift's position using the PIDF controller and the voltage scaler.
     */
    fun update() {
        val voltageCorrection = voltageScaler.voltageCorrection

        val correction =
            liftPID.calculate(liftMotor.currentPosition.toDouble(), liftHeight + voltageCorrection)

        liftMotor.set(correction)
    }

    /**
     * Log data about the lift to a telemetry object.
     *
     * @param telemetry a [Telemetry] object to log data to
     * @param dataSupplier a [DataSupplier] that provides data about the lift motor
     */
    fun logData(telemetry: Telemetry, dataSupplier: DataSupplier<Motor>) {
        telemetry.addData("Lift motor", dataSupplier(liftMotor))
    }
}
