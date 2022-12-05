package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

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
}

class Lift(hwMap: HardwareMap, private val voltageScaler: VoltageScaler) {
    private val motors: Triple<Motor, Motor, Motor>

    private var liftHeight = 0
    private var prevLiftHeight = 0
    private val liftPID: PIDFController

    init {
        motors = Triple(
            Motor(hwMap, "L1", Motor.GoBILDA.RPM_435),
            Motor(hwMap, "L2", Motor.GoBILDA.RPM_435),
            Motor(hwMap, "L3", Motor.GoBILDA.RPM_435),
        )

        transformMotors {
            it.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
            it.setRunMode(Motor.RunMode.VelocityControl)
            it.resetEncoder()
        }

        motors.second.inverted = true

        liftPID = PIDFController(
            LiftConfig.P, LiftConfig.I,
            LiftConfig.D, LiftConfig.F
        )
    }

    fun setFloating() = transformMotors {
        it.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)
    }

    fun setBrake() = transformMotors {
        it.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
    }

    fun goToZero() {
        prevLiftHeight = liftHeight
        liftHeight = LiftConfig.ZERO
    }

    fun goToLow() {
        prevLiftHeight = liftHeight
        liftHeight = LiftConfig.LOW
    }

    fun goToMid() {
        prevLiftHeight = liftHeight
        liftHeight = LiftConfig.MID
    }

    fun goToHigh() {
        prevLiftHeight = liftHeight
        liftHeight = LiftConfig.HIGH
    }

    fun manualLiftControl(power: Double) {
        transformMotors {
            it.setRunMode(Motor.RunMode.RawPower)
            it.set(power)
        }
    }

    var height: Int
        get() = liftHeight
        set(height) {
            liftHeight = Math.max(0, Math.min(height, 3000))
        }

    private fun transformMotors(transformation: (Motor) -> Unit) =
        motors.toList().forEach(transformation)

    fun update() {
        val voltageCorrection = voltageScaler.voltageCorrection

        val correction = liftPID.calculate(motors.first.currentPosition.toDouble(), liftHeight + voltageCorrection)

        transformMotors {
            it.set(correction)
        }
    }
}