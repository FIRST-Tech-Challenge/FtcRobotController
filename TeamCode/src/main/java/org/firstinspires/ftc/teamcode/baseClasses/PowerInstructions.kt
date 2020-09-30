package org.firstinspires.ftc.teamcode.baseClasses

import com.qualcomm.robotcore.util.Range

class PowerInstructions private constructor(
        frontLeft: Double,
        frontRight: Double,
        backLeft: Double,
        backRight: Double
) {
    private val frontLeft = Range.clip(frontLeft, -1.0, 1.0)
    private val frontRight = Range.clip(frontRight, -1.0, 1.0)
    private val backLeft = Range.clip(backLeft, -1.0, 1.0)
    private val backRight = Range.clip(backRight, -1.0, 1.0)

    companion object {
        @JvmStatic fun fromDriveParams(params: MecanumDriveParameters): PowerInstructions {
            return PowerInstructions(
                    frontLeft = params.forwardAmount - params.strafeAmount - params.turnStickX,
                    frontRight = params.forwardAmount + params.strafeAmount + params.turnStickX,
                    backLeft = params.forwardAmount + params.strafeAmount - params.turnStickX,
                    backRight = params.forwardAmount - params.strafeAmount + params.turnStickX
            )
        }
    }

    fun applyOnHardware(hardware: RobotHardware) {
        hardware.base.leftFront.power = frontLeft
        hardware.base.rightFront.power = frontRight
        hardware.base.leftBack.power = backLeft
        hardware.base.rightBack.power = backRight
    }
}