package org.firstinspires.ftc.teamcode.subsystem.turret

import com.pedropathing.controllers.PIDController
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystem.TaskManager
import kotlin.math.abs

class CRTurret(
    taskManager: TaskManager,
    val servoLeft: CRServo,
    val servoMid: CRServo,
    val servoRight: CRServo,
    val encoder: DcMotorEx,
    ) : Turret() {
    companion object {
        val pidCoefficients: PIDCoefficients = PIDCoefficients()
        const val ENCODER_TO_ANGLE_RATIO: Double = 1 / 360.0
    }
    init {
        taskManager.loop.action += { updatePower() }
    }
    val pidController: PIDController = PIDController(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d)
    var power = 0.0
        set(power) {
            field = power
            servoLeft.power = power
            servoMid.power = power
            servoRight.power = power
        }
    override val isWithinTolerance: Boolean
        get() = abs(angleError) < ANGLE_TOLERANCE
    val angleError: Double
        get() = encoder.currentPosition / ENCODER_TO_ANGLE_RATIO - angle
    fun updatePower() {
        val pidOutput = pidController.calculate(angle, angleError)
        power = pidOutput
    }
    override fun printState(telemetry: Telemetry) {
        TODO("Not yet implemented")
    }
}
