package teamcode.v1

import com.acmerobotics.dashboard.config.Config
import com.asiankoala.koawalib.control.controller.PIDGains
import com.asiankoala.koawalib.control.motor.FFGains
import com.asiankoala.koawalib.control.profile.MotionConstraints
import com.asiankoala.koawalib.hardware.motor.EncoderFactory
import com.asiankoala.koawalib.hardware.motor.MotorFactory
import com.asiankoala.koawalib.hardware.servo.KServo
import teamcode.v1.constants.ArmConstants
import teamcode.v1.constants.ClawConstants
import teamcode.v1.constants.LiftConstants
import teamcode.v1.subsystems.KLimitSwitch

class Hardware() {
    val fl = MotorFactory("fl")
        .reverse
        .brake
        .build()

    val bl = MotorFactory("bl")
        .reverse
        .brake
        .build()

    val br = MotorFactory("br")
        .forward
        .brake
        .build()

    val fr = MotorFactory("fr")
        .forward
        .brake
        .build()

    val liftLeadMotor = MotorFactory("liftLead")
        .float
        .reverse
        .createEncoder(EncoderFactory(LiftConstants.ticksPerUnit)
            .zero(LiftConstants.homePos)
            .reverse
        )
        .withPositionControl(
            PIDGains(LiftConstants.kP, LiftConstants.kI, LiftConstants.kD),
            FFGains(kS = LiftConstants.kS, kV = LiftConstants.kV, kA = LiftConstants.kA, kG = LiftConstants.kG),
//            MotionConstraints(LiftConstants.maxVel, LiftConstants.maxAccel),
            allowedPositionError = LiftConstants.allowedPositionError,
            disabledPosition = LiftConstants.disabledPosition
        )
        .build()

    val liftSecondMotor = MotorFactory("lift2")
        .reverse
        .float
        .build()

    val armMotor = MotorFactory("Arm")
        .reverse
        .float
        .createEncoder(EncoderFactory(ArmConstants.ticksPerUnit)
            .reverse
            .zero(ArmConstants.homePos)
        )
        .withMotionProfileControl(
            PIDGains(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD),
            FFGains(kS = ArmConstants.kS, kV = ArmConstants.kV, kA = ArmConstants.kA, kCos = ArmConstants.kCos),
            MotionConstraints(ArmConstants.maxVel, ArmConstants.maxAccel),
            allowedPositionError = ArmConstants.allowedPositionError,
            disabledPosition = ArmConstants.disabledPosition
        )
        .build()

    val clawServo = KServo("Claw")
        .startAt(ClawConstants.closePos)

    val limitSwitch = KLimitSwitch("LimitSwitch")

    @Config
    companion object {
        private const val ticksPerUnit = 1892.3724
    }
}