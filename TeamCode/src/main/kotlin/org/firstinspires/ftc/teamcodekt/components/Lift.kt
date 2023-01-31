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
import ftc.rogue.blacksmith.BlackOp.Companion.hwMap
import ftc.rogue.blacksmith.util.kt.clamp
import ftc.rogue.blacksmith.util.kt.invoke
import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames

@JvmField var LIFT_ZERO = 0
@JvmField var LIFT_LOW = 707
@JvmField var LIFT_MID = 1140
@JvmField var LIFT_HIGH = 1590

@JvmField var LIFT_P = 0.0115
@JvmField var LIFT_I = 0.0002
@JvmField var LIFT_D = 0.0002

@JvmField var LIFT_MAX_V = 1300.0
@JvmField var LIFT_MAX_A = 900.0
@JvmField var LIFT_MAX_J = 600.0

class Lift {
    private val liftMotor = hwMap<DcMotorSimple>(DeviceNames.LIFT_MOTOR)

    private val liftPID = PIDFController(PIDCoefficients(LIFT_P, LIFT_I, LIFT_D))

    private val liftEncoder = Motor(hwMap, DeviceNames.LIFT_ENCODER)
        .apply(Motor::resetEncoder)

    private lateinit var profile: MotionProfile
    init {
        regenMotionProfile()
    }

    private var motionTime = ElapsedTime()

    var targetHeight = 0

    var clippedHeight: Int
        get() = targetHeight
        set(height) {
            targetHeight = height.clamp(LIFT_ZERO, LIFT_HIGH)
        }

    fun goToZero() {
        targetHeight = LIFT_ZERO
        regenMotionProfile()
    }

    fun goToLow() {
        targetHeight = LIFT_LOW
        regenMotionProfile()
    }

    fun goToMid() {
        targetHeight = LIFT_MID
        regenMotionProfile()
    }

    fun goToHigh() {
        targetHeight = LIFT_HIGH
        regenMotionProfile()
    }

    fun goToAngledLow() {
        targetHeight = LIFT_LOW - 145
        regenMotionProfile()
    }

    fun goToAngledMid() {
        targetHeight = LIFT_MID - 190
        regenMotionProfile()
    }

    fun update() {
        val state = profile[motionTime.microseconds()]

        liftPID.apply {
            targetPosition = state.x
            targetVelocity = state.v
            targetAcceleration = state.a
        }

        val correction = liftPID.update(liftHeight.toDouble(), liftVelocity)
        liftMotor.power = correction
    }

    fun resetEncoder() {
        liftEncoder.resetEncoder()
    }

    private var twoPrevVel = 0.0
    private var onePrevVel = 0.0

    private var twoPrevTime = 0L
    private var onePrevTime = 0L

    private fun regenMotionProfile() {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(liftHeight.toDouble(), liftVelocity, liftAccel),
            MotionState(targetHeight.toDouble(), 0.0, 0.0),
            LIFT_MAX_V, LIFT_MAX_A, LIFT_MAX_J
        )
        motionTime.reset()
    }

    private val liftHeight: Int
        get() = liftEncoder.currentPosition

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
            if(twoPrevTime == 0L || onePrevTime == 0L)
                return 0.0

            return 1000 * ((onePrevVel - twoPrevVel) / (onePrevTime - twoPrevTime))
        }

    private fun ElapsedTime.microseconds(): Double {
        return this.milliseconds() / 1000
    }
}
