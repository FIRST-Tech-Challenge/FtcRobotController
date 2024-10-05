package org.firstinspires.ftc.teamcode.opmode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmode.OpModeBase

@Suppress("UNUSED")
@TeleOp(name="CDTeleop")
class CDTeleop : OpModeBase() {
    private var driveSpeedScale = DRIVE_SPEED_NORMAL

    override fun initialize() {
        initHardware()
        initializeDriverGamepad(driverGamepad)
        initializeCoDriverGamepad(accessoryGamepad)
    }

    override fun run() {
        super.run()

        mecanumDrive.setDrivePower(
            Pose2d(
                driverGamepad.leftY * driveSpeedScale,
                -driverGamepad.leftX * driveSpeedScale,
                -driverGamepad.rightX * driveSpeedScale
            )
        )

        mecanumDrive.updatePoseEstimate()

        // TODO: Assign gamepad buttons for raw commands

        writeTelemetry()
    }

    private fun initializeDriverGamepad(gamepad: GamepadEx) {
        val speedFastButton = gamepad.getGamepadButton(GamepadKeys.Button.Y)
        val speedSlowButton = gamepad.getGamepadButton(GamepadKeys.Button.A)
        val normalDriveButton = gamepad.getGamepadButton(GamepadKeys.Button.B)

        speedFastButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_FAST })
        speedSlowButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_SLOW })
        normalDriveButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_NORMAL})
    }

    private fun initializeCoDriverGamepad(gamepad: GamepadEx) {
        // TODO: Figure out what belongs here
    }

    private fun writeTelemetry() {
        telemetry.addLine()
        telemetry.addLine("speed mult: $driveSpeedScale")
        telemetry.addLine()

        // TODO: Refactor this pattern into a helper function
//        hardware.suspendMotor?.let {
//            telemetry.addLine("suspend motor pos: ${it.currentPosition}")
//        } ?: telemetry.addLine("[WARNING] Suspend motor not found")
//
//        hardware.viperAngleServo?.let {
//            telemetry.addLine("viper angle pos: ${it.position}")
//        } ?: telemetry.addLine("[WARNING] Viper angle servo not found")
//
//        hardware.viperPot?.let {
//            telemetry.addLine("pot voltage: ${it.voltage}")
//        } ?: telemetry.addLine("[WARNING] Viper potentiometer not found")
//
//        hardware.viperMotor?.let {
//            telemetry.addLine("viper motor pos: ${it.currentPosition}")
//        } ?: telemetry.addLine("[WARNING] Viper motor not found")
//
//        hardware.droneServo?.let {
//            telemetry.addLine("drone pos: ${it.position}")
//        } ?: telemetry.addLine("[WARNING] Drone servo not found")
//
//        hardware.suspendServo?.let {
//            telemetry.addLine("suspendServo pos: ${it.position}")
//        } ?: telemetry.addLine("[WARNING] suspendServo not found")

        telemetry.update()
    }

    companion object {
        private const val VARIABLE_INPUT_DEAD_ZONE = 0.05

        private const val DRIVE_SPEED_FAST = 0.9
        private const val DRIVE_SPEED_NORMAL = 0.75
        private const val DRIVE_SPEED_SLOW = 0.5
    }
}