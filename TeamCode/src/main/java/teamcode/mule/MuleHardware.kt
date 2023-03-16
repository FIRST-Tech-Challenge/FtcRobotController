package teamcode.mule

import com.asiankoala.koawalib.control.controller.PIDGains
import com.asiankoala.koawalib.control.motor.FFGains
import com.asiankoala.koawalib.control.profile.MotionConstraints
import com.asiankoala.koawalib.hardware.motor.EncoderFactory
import com.asiankoala.koawalib.hardware.motor.MotorFactory
import com.asiankoala.koawalib.hardware.sensor.KDistanceSensor
import com.asiankoala.koawalib.hardware.servo.KServo
import teamcode.v1.constants.ClawConstants
import teamcode.mule.constants.MuleArmConstants

class MuleHardware {
    val armMotor = MotorFactory("Arm")
            .float
            .createEncoder(EncoderFactory(MuleArmConstants.ticksPerUnit)
                    .zero(MuleArmConstants.homePos)
            )
            .withMotionProfileControl(
                    PIDGains(MuleArmConstants.kP, MuleArmConstants.kI, MuleArmConstants.kD),
                    FFGains(kS = MuleArmConstants.kS, kV = MuleArmConstants.kV, kA = MuleArmConstants.kA, kCos = MuleArmConstants.kCos),
                    MotionConstraints(MuleArmConstants.maxVel, MuleArmConstants.maxAccel),
                    allowedPositionError = MuleArmConstants.allowedPositionError
            )
            .build()

        val clawServo = KServo("Claw")
                .startAt(ClawConstants.openPos)

        val distanceSensor = KDistanceSensor("dSensor")
}