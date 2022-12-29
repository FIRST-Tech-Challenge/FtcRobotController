package teamcode.mule

import com.acmerobotics.dashboard.config.Config
import com.asiankoala.koawalib.control.controller.PIDGains
import com.asiankoala.koawalib.control.motor.FFGains
import com.asiankoala.koawalib.control.profile.MotionConstraints
import com.asiankoala.koawalib.hardware.motor.EncoderFactory
import com.asiankoala.koawalib.hardware.motor.KEncoder
import com.asiankoala.koawalib.hardware.motor.MotorFactory
import com.asiankoala.koawalib.math.Pose
import teamcode.mule.constants.MuleArmConstants

class MuleHardware {
    val armMotor = MotorFactory("Arm")
            .reverse
            .float
            .createEncoder(EncoderFactory(MuleArmConstants.ticksPerUnit)
                    .reverse
                    .zero(MuleArmConstants.homePos)
            )
            .withPositionControl(
                    PIDGains(MuleArmConstants.kP, MuleArmConstants.kI, MuleArmConstants.kD),
                    FFGains(kS = MuleArmConstants.kS, kV = MuleArmConstants.kV, kA = MuleArmConstants.kA, kCos = MuleArmConstants.kCos),
                    allowedPositionError = MuleArmConstants.allowedPositionError
            )
            .build()

}