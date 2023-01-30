package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import ftc.rogue.blacksmith.util.kt.clamp
import ftc.rogue.blacksmith.util.kt.invoke
import org.firstinspires.ftc.teamcodekt.components.LiftConfig.MAX_A
import org.firstinspires.ftc.teamcodekt.components.LiftConfig.MAX_J
import org.firstinspires.ftc.teamcodekt.components.LiftConfig.MAX_V
import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames

@Config
object LiftConfig {
    @JvmField
    var P = 0.0115
    @JvmField
    var I = 0.0002
    @JvmField
    var D = 0.0002
//    @JvmField
//    var F = 0.00002

    @JvmField
    var ZERO = 0
    @JvmField
    var LOW = 707
    @JvmField
    var MID = 1140
    @JvmField
    var HIGH = 1590

    @JvmField
    var MAX_V = 1300.0
    @JvmField
    var MAX_A = 900.0
    @JvmField
    var MAX_J = 600.0
}

class Lift {
    private val liftMotor = hwMap<DcMotorSimple>(DeviceNames.LIFT_MOTOR)

    private val voltageScaler = VoltageScaler(hwMap)

    private val liftPID = PIDFController(PIDCoefficients(
            kP = LiftConfig.P,
            kI = LiftConfig.I,
            kD = LiftConfig.D))
    private val liftEncoder = Motor(hwMap, DeviceNames.LIFT_ENCODER)
            .apply(Motor::resetEncoder)

    private var targetHeight = 0

    private var twoPrevVel = 0.0
    private var onePrevVel = 0.0

    private var twoPrevTime = 0.toLong()
    private var onePrevTime = 0.toLong()

    private var profile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(getLiftHeight().toDouble(), getLiftVelocity(), 0.0),
            MotionState(targetHeight.toDouble(), 0.0, 0.0),
            MAX_V,
            MAX_A,
            MAX_J
    )

    private var motionTime = ElapsedTime();

    var height: Int
        get() = targetHeight
        set(height) {
            targetHeight = height.clamp(LiftConfig.ZERO, LiftConfig.HIGH)
        }

    var rawHeight: Int
        get() = targetHeight
        set(height) {
            targetHeight = height
        }

    fun goToZero() {
        targetHeight = LiftConfig.ZERO
        setMotionProfile()
    }

    fun goToLow() {
        targetHeight = LiftConfig.LOW
        setMotionProfile()
    }

    fun goToMid() {
        targetHeight = LiftConfig.MID
        setMotionProfile()
    }

    fun goToAngledMid() {
        targetHeight = LiftConfig.MID - 190
        setMotionProfile()
    }

    fun goToAngledLow() {
        targetHeight = LiftConfig.LOW - 145
        setMotionProfile()
    }

    fun goToHigh() {
        targetHeight = LiftConfig.HIGH
        setMotionProfile()
    }

    fun setMotionProfile() {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(getLiftHeight().toDouble(), getLiftVelocity(), getLiftAccel()),
                MotionState(targetHeight.toDouble(), 0.0, 0.0),
                MAX_V,
                MAX_A,
                MAX_J
        )
        motionTime.reset()
    }

    fun update(liftDeadzone: Double) {
        val state = profile[motionTime.milliseconds()/1000]
        liftPID.apply {
            targetPosition = state.x
            targetVelocity = state.v
            targetAcceleration = state.a
        }

        val correction =
            liftPID.update(measuredPosition = getLiftHeight().toDouble(), measuredVelocity = getLiftVelocity())
        liftMotor.power = correction
//
//        if(abs(correction) < liftDeadzone)
//            correction = 0.0
//
//        mTelemetry.addData("Lift correction", correction)
//        mTelemetry.addData("Lift position", -liftEncoder.currentPosition)
//
//        liftMotor.power = correction
    }

    fun resetEncoder() {
        liftEncoder.resetEncoder()
    }

    fun getLiftHeight(): Int {
        return liftEncoder.currentPosition
    }

    fun getLiftVelocity(): Double {
        twoPrevVel = onePrevVel
        onePrevVel = liftEncoder.correctedVelocity

        twoPrevTime = onePrevTime
        onePrevTime = System.currentTimeMillis()
        return liftEncoder.correctedVelocity
    }

    private fun getLiftAccel(): Double {
        // TODO: Check if this actually works. a bit sus imo but idk might work or be close enough
        if(twoPrevTime == 0.toLong() || onePrevTime == 0.toLong())
            return 0.0

        return (onePrevVel-twoPrevVel)/(onePrevTime/1000-twoPrevTime/1000)
    }
}
