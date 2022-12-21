package org.firstinspires.ftc.teamcodekt.components

import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import ftc.rouge.blacksmith.util.kt.clamp
import ftc.rouge.blacksmith.util.kt.invoke
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotConstants.LiftConfig
import org.firstinspires.ftc.teamcodekt.util.DataSupplier

/**
 * This class represents a lift that can be moved to different heights.
 *
 * @param hwMap a [HardwareMap] object that contains information about the robot's hardware
 * @param voltageScaler a [VoltageScaler] object that helps correct for voltage changes
 * @author KG
 */
class Lift(hwMap: HardwareMap, private val voltageScaler: VoltageScaler) {
    private val liftMotor: DcMotorSimple

    val liftEncoder: Motor
    private val liftPID: PIDFController

    var targetHeight = 0

    /**
     * The height of the lift. This property is clamped between [LiftConfig.ZERO] and
     * [LiftConfig.HIGH]. Only should really be used for manual control.
     */
    var height: Int
        get() = targetHeight
        set(height) {
            targetHeight = height.clamp(LiftConfig.ZERO, LiftConfig.HIGH)
        }

    init {
        liftMotor = hwMap("LI")

        liftEncoder = Motor(hwMap, "FR")
        liftEncoder.resetEncoder()

        liftPID = PIDFController(
            LiftConfig.P, LiftConfig.I,
            LiftConfig.D, LiftConfig.F
        )
    }

    /**
     * Move the lift to the zero height. Used for intaking or depositing on ground junctions/terminals.
     */
    fun goToZero() {
        targetHeight = LiftConfig.ZERO
    }

    /**
     * Move the lift to the low height. Used for low poles.
     */
    fun goToLow() {
        targetHeight = LiftConfig.LOW
    }

    /**
     * Move the lift to the mid height. Used for mid poles.
     */
    fun goToMid() {
        targetHeight = LiftConfig.MID
    }

    /**
     * Move the lift to the high height. Used for high poles.
     */
    fun goToHigh() {
        targetHeight = LiftConfig.HIGH
    }

    /**
     * Update the lift's position using the PIDF controller and the voltage scaler.
     */
    fun update(telemetry: Telemetry) {
        val voltageCorrection = voltageScaler.voltageCorrection

        val correction =
            liftPID.calculate(-liftEncoder.currentPosition.toDouble(), targetHeight.toDouble())

        telemetry.addData("Corection", correction)

        liftMotor.power = correction
    }

    /**
     * Log data about the lift to a telemetry object.
     *
     * @param telemetry a [Telemetry] object to log data to
     * @param dataSupplier a [DataSupplier] that provides data about the lift motor
     */
    fun logData(telemetry: Telemetry, dataSupplier: DataSupplier<Lift>) {
        telemetry.addData("Lift motor", dataSupplier(this))
    }
}
