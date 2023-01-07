package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import ftc.rogue.blacksmith.util.kt.clamp
import ftc.rogue.blacksmith.util.kt.invoke
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcodekt.util.DataSupplier
import kotlin.math.abs

@Config
object LiftConfig {
    @JvmField var P = 0.0115
    @JvmField var I = 0.0002
    @JvmField var D = 0.0002
    @JvmField var F = 0.00002

    @JvmField var ZERO = 0
    @JvmField var LOW  = 737
    @JvmField var MID  = 1170
    @JvmField var HIGH = 1550

    @JvmField var MANUAL_ADJUSTMENT_MULTI = 50.0
}

class Lift {
    private val liftMotor = hwMap<DcMotorSimple>(DeviceNames.LIFT_MOTOR)
    
    private val voltageScaler = VoltageScaler(hwMap)
    
    private val liftPID = PIDFController(LiftConfig.P, LiftConfig.I, LiftConfig.D, LiftConfig.F)

    private val liftEncoder = Motor(hwMap, DeviceNames.LIFT_ENCODER)
        .apply(Motor::resetEncoder)

    private var targetHeight = 0

    var height: Int
        get() = targetHeight
        set(height) {
            targetHeight = height.clamp(LiftConfig.ZERO, LiftConfig.HIGH)
        }

    fun goToZero() {
        targetHeight = LiftConfig.ZERO
    }

    fun goToLow() {
        targetHeight = LiftConfig.LOW
    }

    fun goToMid() {
        targetHeight = LiftConfig.MID
    }

    fun goToHigh() {
        targetHeight = LiftConfig.HIGH
    }

    fun update() {
        val voltageCorrection = voltageScaler.voltageCorrection

        var correction =
            liftPID.calculate(-liftEncoder.currentPosition.toDouble(), targetHeight + voltageCorrection)

        if(abs(correction) < 0.05)
            correction = 0.0

        liftMotor.power = correction
    }

    fun logData(telemetry: Telemetry, dataSupplier: DataSupplier<Lift>) {
        telemetry.addData("Lift motor", dataSupplier(this))
    }
}
