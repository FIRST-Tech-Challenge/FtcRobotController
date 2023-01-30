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

@JvmField var P = 0.0115
@JvmField var I = 0.0002
@JvmField var D = 0.0002

@JvmField var ZERO = 0
@JvmField var LOW = 707
@JvmField var MID = 1140
@JvmField var HIGH = 1590

@JvmField var MAX_V = 1300.0
@JvmField var MAX_A = 900.0
@JvmField var MAX_J = 600.0

class Lift {
    private val liftMotor = hwMap<DcMotorSimple>(DeviceNames.LIFT_MOTOR)

    private val liftPID = PIDFController(PIDCoefficients(P, I, D))

    private val liftEncoder = Motor(hwMap, DeviceNames.LIFT_ENCODER)
        .apply(Motor::resetEncoder)

    private var twoPrevVel = 0.0
    private var onePrevVel = 0.0

    private var twoPrevTime = 0L
    private var onePrevTime = 0L

    private var motionTime = ElapsedTime()

    private lateinit var profile: MotionProfile
    init {
        regenMotionProfile()
    }

    var targetHeight = 0

    var clippedHeight: Int
        get() = targetHeight
        set(height) {
            targetHeight = height.clamp(ZERO, HIGH)
        }

    fun goToZero() {
        targetHeight = ZERO
        regenMotionProfile()
    }

    fun goToLow() {
        targetHeight = LOW
        regenMotionProfile()
    }

    fun goToMid() {
        targetHeight = MID
        regenMotionProfile()
    }

    fun goToHigh() {
        targetHeight = HIGH
        regenMotionProfile()
    }

    fun goToAngledLow() {
        targetHeight = LOW - 145
        regenMotionProfile()
    }

    fun goToAngledMid() {
        targetHeight = MID - 190
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

    private fun regenMotionProfile() {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(liftHeight.toDouble(), liftVelocity, liftAccel),
            MotionState(targetHeight.toDouble(), 0.0, 0.0),
            MAX_V, MAX_A, MAX_J
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
        get()  {
            // TODO: Check if this actually works. a bit sus imo but idk might work or be close enough
            if(twoPrevTime == 0L || onePrevTime == 0L)
                return 0.0

            return 1000 * ((onePrevVel - twoPrevVel) / (onePrevTime - twoPrevTime))
        }

    private fun ElapsedTime.microseconds(): Double {
        return this.milliseconds() / 1000
    }
}
