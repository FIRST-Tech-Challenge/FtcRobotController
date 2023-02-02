@file:Config

package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import ftc.rogue.blacksmith.BlackOp
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import ftc.rogue.blacksmith.util.kt.clamp
import ftc.rogue.blacksmith.util.kt.invoke
import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames
import org.firstinspires.ftc.teamcodekt.util.KalmanFilter

@JvmField
var LIFT_ZERO = 0
@JvmField
var LIFT_LOW = 707
@JvmField
var LIFT_MID = 1140
@JvmField
var LIFT_HIGH = 1590


@JvmField
var NORMAL_LIFT_P = 0.0025


@JvmField
var NORMAL_LIFT_I = 0.01
@JvmField
var NORMAL_LIFT_D = 0.0002
@JvmField
var NORMAL_LIFT_F = 0.00002

@JvmField
var MOTION_PROFILE_LIFT_P = 0.026
@JvmField
var MOTION_PROFILE_LIFT_I = 0.0002
@JvmField
var MOTION_PROFILE_LIFT_D = 0.0002

@JvmField
var LIFT_MAX_V = 32000.0
@JvmField
var LIFT_MAX_A = 25000.0
@JvmField
var LIFT_MAX_J = 18000.0

@JvmField
var PROCESS_NOISE = 0.01
@JvmField
var MEASUREMENT_NOISE = 0.01

class Lift(val usingMotionProfiling: Boolean) {

    private val liftMotor = hwMap<DcMotorSimple>(DeviceNames.LIFT_MOTOR)

    private val liftPID = PIDFController(PIDCoefficients(MOTION_PROFILE_LIFT_P, MOTION_PROFILE_LIFT_I, MOTION_PROFILE_LIFT_D))

    val normalPID = com.arcrobotics.ftclib.controller.PIDFController(NORMAL_LIFT_P, NORMAL_LIFT_I, NORMAL_LIFT_D, NORMAL_LIFT_F)

    val liftFilter = KalmanFilter(PROCESS_NOISE, MEASUREMENT_NOISE)

    private val liftEncoder = Motor(hwMap, DeviceNames.LIFT_ENCODER)
            .apply(Motor::resetEncoder)

    private lateinit var profile: MotionProfile

    init {
        if(usingMotionProfiling)
            regenMotionProfile(0)
    }

    private var motionTime = ElapsedTime()

    var targetHeight = 0
        set(height) {
            field = height
            if(usingMotionProfiling)
                regenMotionProfile(height)
        }


    var clippedHeight: Int
        get() = targetHeight
        set(height) {
            targetHeight = height.clamp(LIFT_ZERO, LIFT_HIGH)
        }

    fun goToZero() {
        targetHeight = LIFT_ZERO
    }

    fun goToLow() {
        targetHeight = LIFT_LOW
    }

    fun goToMid() {
        targetHeight = LIFT_MID
    }

    fun goToHigh() {
        targetHeight = LIFT_HIGH
    }

    fun goToAngledHigh() {
        targetHeight = LIFT_HIGH - 356
    }

    fun goToAngledLow() {
        targetHeight = LIFT_LOW - 360
    }

    fun goToAngledMid() {
        targetHeight = LIFT_MID - 350
    }


    fun update() {
        if (usingMotionProfiling) {
            val state = profile[motionTime.microseconds()]

            liftPID.apply {
                targetPosition = state.x
                targetVelocity = state.v
                targetAcceleration = state.a
            }

            var correction = liftPID.update(liftHeight.toDouble(), liftVelocity)
            if(liftHeight < 10 && targetHeight == LIFT_ZERO)
                correction = 0.0
            liftMotor.power = correction
        } else {
            val correction = normalPID.calculate(liftHeight.toDouble(), targetHeight.toDouble())
            liftMotor.power = liftFilter.filter(liftMotor.power + correction)
        }
    }

    fun resetEncoder() {
        liftEncoder.resetEncoder()
    }

    private var twoPrevVel = 0.0
    private var onePrevVel = 0.0

    private var twoPrevTime = 0L
    private var onePrevTime = 0L

    private fun regenMotionProfile(targetHeight: Int) {
        try {
            if (motionTime == null)
                motionTime = ElapsedTime(0);
            motionTime.reset()
            profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    MotionState(liftHeight.toDouble(), liftVelocity, liftAccel),
                    MotionState(targetHeight.toDouble(), 0.0, 0.0),
                    LIFT_MAX_V, LIFT_MAX_A, LIFT_MAX_J
            )
        }
        catch (e: Exception){
            return
        }

    }

    private val liftHeight: Int
        get() = -liftEncoder.currentPosition

    private val liftVelocity: Double
        get() {
            twoPrevVel = onePrevVel
            onePrevVel = liftEncoder.correctedVelocity

            twoPrevTime = onePrevTime
            onePrevTime = System.currentTimeMillis()
            return liftEncoder.correctedVelocity
        }


    private val liftAccel: Double
        get() {
            // TODO: Check if this actually works. a bit sus imo but idk might work or be close enough
            if (twoPrevTime == 0L || onePrevTime == 0L)
                return 0.0

            return 1000 * ((onePrevVel - twoPrevVel) / (onePrevTime - twoPrevTime))
        }

    fun printLiftTelem( ) {
        BlackOp.mTelemetry.addData("Current lift height:", liftHeight)
        BlackOp.mTelemetry.addData("Lift target height:", targetHeight)
    }

    private fun ElapsedTime.microseconds(): Double {
        return this.milliseconds() / 1000
    }
}
