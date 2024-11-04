package org.firstinspires.ftc.teamcode.util

import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.opmodes.logger

fun IntRange.shift(n: Int) = first + n..last + n

fun ToggleButtonReader.check() = this.readValue().let { this.state }

object Constants {

    object DriveTrain {
        const val LOW_SPEED = 0.4
        const val HIGH_SPEED = 0.8
    }
}

class DriveTrain(private val hardware: RobotHardware) : MecanumDrive(
    hardware.frontLeftMotor,
    hardware.frontRightMotor,
    hardware.backLeftMotor,
    hardware.backRightMotor,
) {
    fun drive(speed: Double? = null) {
        val speedModifier = speed
            ?: if (hardware.toggleFast.check()) Constants.DriveTrain.HIGH_SPEED else Constants.DriveTrain.LOW_SPEED

        val driveSpeed = if (speedModifier == Constants.DriveTrain.HIGH_SPEED) "High" else "Low"
        logger.debug("Drive Speed: $driveSpeed")

        driveRobotCentric(
            -hardware.gamepad.leftX * speedModifier,
            -hardware.gamepad.leftY * speedModifier,
            -hardware.gamepad.rightX * speedModifier,
        )
    }
}

class RobotHardware(val hardwareMap: HardwareMap, val gamepad: GamepadEx) {
    val frontLeftMotor = Motor(hardwareMap, "flMotor", Motor.GoBILDA.RPM_435)
    val frontRightMotor = Motor(hardwareMap, "frMotor", Motor.GoBILDA.RPM_435)
    val backLeftMotor = Motor(hardwareMap, "blMotor", Motor.GoBILDA.RPM_435)
    val backRightMotor = Motor(hardwareMap, "brMotor", Motor.GoBILDA.RPM_435)
    val driveTrain = DriveTrain(this)

    val toggleFast = ToggleButtonReader(gamepad, GamepadKeys.Button.RIGHT_STICK_BUTTON)
    val toggleHang = ToggleButtonReader(gamepad, GamepadKeys.Button.BACK)
}