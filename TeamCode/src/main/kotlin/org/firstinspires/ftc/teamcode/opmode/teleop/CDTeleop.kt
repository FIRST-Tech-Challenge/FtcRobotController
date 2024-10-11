package org.firstinspires.ftc.teamcode.opmode.teleop

import android.annotation.SuppressLint
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmode.OpModeBase
import kotlin.math.pow

@Suppress("UNUSED")
@TeleOp(name="CDTeleop")
class CDTeleop : OpModeBase() {
    private var driveSpeedScale = DRIVE_SPEED_NORMAL

    override fun initialize() {
        initHardware()
        initializeDriverGamepad(driverGamepad)
        initializeCoDriverGamepad(accessoryGamepad)
    }

    @SuppressLint("UseValueOf")
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

        // Driver controls
        val driverLeftTriggerValue = driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        val driverRightTriggerValue = driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)

        if (hardware.gripperHomeSensor?.isPressed == true) {
            hardware.gripperServo?.power = 0.0
        } else if (driverLeftTriggerValue > VARIABLE_INPUT_DEAD_ZONE) {
            hardware.gripperServo?.power = -0.5
        } else if (driverRightTriggerValue > VARIABLE_INPUT_DEAD_ZONE) {
            hardware.gripperServo?.power = 0.5
        } else {
            hardware.gripperServo?.power = 0.0
        }

        // Accessory controls
        val accessoryLeftTriggerValue = accessoryGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        val accessoryRightTriggerValue = accessoryGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)

        if (accessoryLeftTriggerValue > VARIABLE_INPUT_DEAD_ZONE) {
            hardware.intakeWheelServo?.power = -1.0
        } else if (accessoryRightTriggerValue > VARIABLE_INPUT_DEAD_ZONE) {
            hardware.intakeWheelServo?.power = 1.0
        } else {
            hardware.intakeWheelServo?.power = 0.0
        }

        if (accessoryGamepad.rightY > VARIABLE_INPUT_DEAD_ZONE || accessoryGamepad.rightY < -VARIABLE_INPUT_DEAD_ZONE) {
            // TODO: Fix multiplier later
            viperArmSubsystem.setExtensionMotorGroupPower((-accessoryGamepad.rightY).pow(3.0) * 0.5)
        } else {
            viperArmSubsystem.setExtensionMotorGroupPower(0.0)
        }

        if (accessoryGamepad.leftY > VARIABLE_INPUT_DEAD_ZONE || accessoryGamepad.leftY < -VARIABLE_INPUT_DEAD_ZONE) {
            // TODO: Fix multiplier later
            viperArmSubsystem.setRotationMotorGroupPower((-accessoryGamepad.leftY).pow(3.0) * 0.5)
        } else {
            viperArmSubsystem.setRotationMotorGroupPower(0.0)
        }

        writeTelemetry()
    }

    private fun initializeDriverGamepad(gamepad: GamepadEx) {
        val speedFastButton = gamepad.getGamepadButton(GamepadKeys.Button.Y)
        val speedSlowButton = gamepad.getGamepadButton(GamepadKeys.Button.A)
        val normalDriveButton = gamepad.getGamepadButton(GamepadKeys.Button.B)
//        val gripperPickupButton = gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//        val gripperLowDeliveryButton = gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//        val gripperHighDeliveryButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)

        val retractForClimbButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val ratchetClimbButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)

        speedFastButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_FAST })
        speedSlowButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_SLOW })
        normalDriveButton.whenPressed(Runnable { driveSpeedScale = DRIVE_SPEED_NORMAL})
//        lowChamberHeightButton.whenPressed(Runnable { gripperSubsystem.setLowChamberHeight() })

        // TODO: Set correct numbers from telemetry
        retractForClimbButton.whenPressed(Runnable { viperArmSubsystem.extendToPosition(0) })
        ratchetClimbButton.whenPressed(Runnable {
            viperArmSubsystem.rotateToPosition(0)
            viperArmSubsystem.extendToPosition(0)
            viperArmSubsystem.rotateToPosition(200)
        })
    }

    private fun initializeCoDriverGamepad(gamepad: GamepadEx) {
        val wristForwardButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
        val wristReverseButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
        val wristLeftButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
        val wristRightButton = gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)

        wristForwardButton.whenPressed(Runnable { activeIntakeSubsystem.rotateHome() })
        wristReverseButton.whenPressed(Runnable { activeIntakeSubsystem.rotateToBasket() })
//        wristLeftButton.whenPressed(Runnable { activeIntakeSubsystem.rotateLeft() })
//        wristRightButton.whenPressed(Runnable { activeIntakeSubsystem.rotateRight() })

        wristLeftButton.whenPressed(Runnable { activeIntakeSubsystem.rotateIncrementDown() })
        wristRightButton.whenPressed(Runnable { activeIntakeSubsystem.rotateIncrementUp() })
    }

    private fun writeTelemetry() {
        telemetry.addLine()
        telemetry.addLine("speed mult: $driveSpeedScale")
        telemetry.addLine()

        hardware.viperExtensionMotorGroup?.let {
            telemetry.addLine("viperExtensionPosition: ${hardware.viperExtensionMotorGroup?.get() ?: "null"}")
        } ?: telemetry.addLine("[WARNING] viperExtensionGroup not found")

        hardware.viperRotationMotorGroup?.let {
            telemetry.addLine("viperRotationPosition: ${hardware.viperRotationMotorGroup?.get() ?: "null"}")
        } ?: telemetry.addLine("[WARNING] viperRotationGroup not found")

        hardware.intakeRotateServo?.let {
            telemetry.addLine("intakeRotationPosition: ${hardware.intakeRotateServo?.position ?: "null"}")
        } ?: telemetry.addLine("[WARNING] wrist servo not found")

//        telemetry.addLine("viperExtensionPos: ${viperArmSubsystem.getExtensionMotorGroupPosition()}")
//        telemetry.addLine("viperRotationPos: ${viperArmSubsystem.getRotationMotorGroupPosition()}")
//        telemetry.addLine()

        // TODO: Position is meaningless for continuous servos, we need some way to measure position without an encoder in the servo

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