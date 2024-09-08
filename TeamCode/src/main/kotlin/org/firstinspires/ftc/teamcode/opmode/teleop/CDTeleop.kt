package org.firstinspires.ftc.teamcode.opmode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmode.OpModeBase


@TeleOp(name="CDTeleop")
class CDTeleop : OpModeBase() {
    override fun initialize() {
        initHardware(false)

        // TODO: Assign gamepad buttons for subsystem commands

//        GamepadButton(accessoryGamepad, GamepadKeys.Button.LEFT_BUMPER).whenPressed(Command {
//            deposit.goToHeight(
//                28
//            )
//        })
//        GamepadButton(
//            accessoryGamepad,
//            GamepadKeys.Button.RIGHT_BUMPER
//        ).whenPressed(deposit::teleopDeploy)
//        GamepadButton(accessoryGamepad, GamepadKeys.Button.A).whenPressed(deposit::retract)
//        GamepadButton(accessoryGamepad, GamepadKeys.Button.Y).whenPressed(Command {
//            deposit.goToHeight(
//                14.0
//            )
//        })
//        GamepadButton(accessoryGamepad, GamepadKeys.Button.X).whenPressed(Command {
//            deposit.goToHeight(
//                7.0
//            )
//        })
//        GamepadButton(accessoryGamepad, GamepadKeys.Button.B).whenPressed(Command {
//            deposit.goToHeight(
//                10.5
//            )
//        })
//
//        Trigger { gamepad2.left_trigger > 0.3 }.whenActive { deposit.goToHeight(deposit.getTargetHeight() - 1.5) }
//        Trigger { gamepad2.right_trigger > 0.3 }.whenActive { deposit.goToHeight(deposit.getTargetHeight() + 1.5) }
//
//        GamepadButton(
//            accessoryGamepad,
//            GamepadKeys.Button.LEFT_STICK_BUTTON
//        ).whenPressed(Runnable { Deposit.offset += 0.25 })
//        GamepadButton(
//            accessoryGamepad,
//            GamepadKeys.Button.RIGHT_STICK_BUTTON
//        ).whenPressed(Runnable { Deposit.offset -= 0.25 })
//
//        GamepadButton(
//            accessoryGamepad,
//            GamepadKeys.Button.DPAD_LEFT
//        ).whenPressed(turretCap::decreaseTurret)
//        GamepadButton(
//            accessoryGamepad,
//            GamepadKeys.Button.DPAD_RIGHT
//        ).whenPressed(turretCap::increaseTurret)
//
//        GamepadButton(
//            accessoryGamepad,
//            GamepadKeys.Button.DPAD_DOWN
//        ).whenPressed(turretCap::decreaseTilt)
//        GamepadButton(
//            accessoryGamepad,
//            GamepadKeys.Button.DPAD_UP
//        ).whenPressed(turretCap::increaseTilt)
//
//        GamepadButton(accessoryGamepad, GamepadKeys.Button.START).whenPressed(
//            Runnable { deposit.disableBlocker = !deposit.disableBlocker })
//
//        GamepadButton(accessoryGamepad, GamepadKeys.Button.BACK).whenPressed(
//            Runnable { turretCap.isFastMode = !turretCap.isFastMode })
    }

    override fun run() {
        super.run()

        mecanumDrive.setDrivePower(
            Pose2d(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            )
        )

        mecanumDrive.updatePoseEstimate()

        // TODO: Assign gamepad buttons for raw commands

//        val rawIntakePower = -gamepad2.left_stick_y.toDouble()
//        intake.setPower(Math.signum(rawIntakePower) * rawIntakePower * rawIntakePower)
//
//        carousel.setPower(gamepad1.left_trigger - gamepad1.right_trigger)
//
//        turretCap.setExtendPower(-gamepad2.right_stick_y)
    }
}