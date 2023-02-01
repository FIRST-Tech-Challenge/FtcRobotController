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
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames
import kotlin.math.abs

@JvmField
var LIFT_ZERO = 0
@JvmField
var LIFT_LOW = 707
@JvmField
var LIFT_MID = 1140
@JvmField
var LIFT_HIGH = 1590

@JvmField
var LIFT_P = 0.026
@JvmField
var LIFT_I = 0.0002
@JvmField
var LIFT_D = 0.0002

@JvmField
var LIFT_MAX_V = 29000.0
@JvmField
var LIFT_MAX_A = 20000.0
@JvmField
var LIFT_MAX_J = 20000.0

class Lift {
    private val liftMotor = hwMap<DcMotorSimple>(DeviceNames.LIFT_MOTOR)

    private val liftPID = PIDFController(PIDCoefficients(LIFT_P, LIFT_I, LIFT_D))

    private val normalPID = com.arcrobotics.ftclib.controller.PIDFController(0.0019, LIFT_I, LIFT_D, 0.0)

    private val liftEncoder = Motor(hwMap, DeviceNames.LIFT_ENCODER)
            .apply(Motor::resetEncoder)

    private lateinit var profile: MotionProfile

    init {
        regenMotionProfile()
    }

    private var motionTime = ElapsedTime()

    var targetHeight = 0

    var mult = 1

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

    fun goToAngledHigh() {
        targetHeight = LIFT_HIGH - 350
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
        update(true)
    }

    fun update(useMotionProfiling: Boolean) {
        if (useMotionProfiling) {
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
            liftMotor.power += correction
        }
    }

    fun resetEncoder() {
        liftEncoder.resetEncoder()
    }

    private var twoPrevVel = 0.0
    private var onePrevVel = 0.0

    private var twoPrevTime = 0L
    private var onePrevTime = 0L

    fun regenMotionProfile() {
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

    fun printLiftTelem(telemetry: Telemetry) {
        telemetry.addData("Current lift height:", liftHeight)
        telemetry.addData("Lift target height:", targetHeight)
    }

    private fun ElapsedTime.microseconds(): Double {
        return this.milliseconds() / 1000
    }
}
