/**
 * Code is based on https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
 * Mechanum drive TeleOp
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Control Mapping
 *  GAMEPAD1 - Drive & Manual Lift
 *      Left Joystick X - Strafe
 *      Left Joystick Y - Forward Backward
 *      Right Joystick X - Pivot
 *      Down Pad - Pushoff
 *      Up Pad - Pushoff reset
 *      Right Trigger = raise lift
 *      Left Trigger = lower lift
 *      A Button - Lift low sequence [1 stage]
 *      B Button - Lift High sequence [2 stage]
 *
 *
 *  GAMEPAD2 - Arm, Arm Pivot, Intake
 *      Right Trigger - Intake
 *      Left Trigger - Extend Arm
 *      Right Bumper - Outtake
 *      Left Bumper - Retract Arm
 *      Y Button - Raise Arm
 *      A Button - Lower Arm
 *      X Button - reset encoder for arms
 *
 *
 *  NOTE: Low lift must be completed before high lift is engaged. This is due to rules and the sequencing of getting to
 *      the high bar relies on the bot being already hanging from the low bar.
 */

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Bot bot = new Bot(this);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ===== Gamepad 1 =====
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x; // Counteract imperfect strafing
            double pivot = -gamepad1.right_stick_x;

            double liftUpControl = gamepad1.right_trigger;
            double liftDownControl = gamepad1.left_trigger;
            boolean liftSeqControl = gamepad1.a;

            boolean pushoffDownControl = gamepad1.dpad_down;
            boolean pushoffUpControl = gamepad1.dpad_up;

            // =====================

            // ===== Gamepad 2 =====
            double intakeControl = gamepad2.right_trigger;
            boolean outtakeControl = gamepad2.right_bumper;

            double extendOutControl = gamepad2.left_trigger;
            boolean extendInControl = gamepad2.left_bumper;

            boolean pivotUpControl = gamepad2.y;
            boolean pivotDownControl = gamepad2.a;

            boolean resetEncoderControl = gamepad2.x;

            // =====================


            // === INTAKE ===
            if (intakeControl > 0.01) {
                bot.setIntakePosition(-1.0);
            } else if (outtakeControl) {
                bot.setIntakePosition(1.0);
            }
            //when no button is pressed, nothing rotates
            else {
                bot.setIntakePosition(0.0);
            }

            // === DRIVE ===
            //mechanum drive equations for powering each motor
            double frontLeftPower = y + x + pivot;
            double backLeftPower = y - x + pivot;
            double frontRightPower = y - x - pivot;
            double backRightPower = y + x - pivot;

            //normalize power value
            double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }
            bot.setDriveTrain(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

            // === LIFT ===
            if (liftUpControl > 0.1 &&
                    (bot.getRightLiftPos() < Bot.RIGHT_LIFT_MAX && bot.getLeftLiftPos() < Bot.LEFT_LIFT_MAX)) {
                bot.setLift(1.0);//this makes it go up
            } else if (liftDownControl > 0.1 &&
                    bot.getRightLiftPos() > Bot.RIGHT_LIFT_MIN && bot.getLeftLiftPos() > Bot.LEFT_LIFT_MIN) {
                bot.setLift(-1.0);//this makes it go down
            } else {
                bot.setLift(0.0);
            }

            if (liftSeqControl) { // Takes 25 sec to complete low hang
                bot.liftLow();
            }

            // === EXTEND ===
            if (extendOutControl > 0.01 && bot.getExtendPos() >= Bot.MAX_EXT) {
                bot.setExtendPower(-1.0);
            } else if (extendInControl && bot.getExtendPos() <= Bot.MIN_EXTEND) {
                bot.setExtendPower(1.0);
            } else {
                bot.setExtendPower(0.0);
            }

            // === PIVOT ===
            if (pivotUpControl && bot.getArmPosition() <= Bot.MAX_PIVOT) {
                bot.setPivotPower(0.50);
            } else if (pivotDownControl && bot.getArmPosition() >= Bot.MIN_PIVOT) {
                bot.setPivotPower(-0.50);
            } else {
                bot.setPivotPower(0.0);
            }

            // === PUSHOFF ===
            if (pushoffUpControl) {
                bot.setPushoff(1.0);
            } else if (pushoffDownControl) {
                bot.setPushoff(-1.0);
            } else {
                bot.setPushoff(0.0);
            }

            // === RESET ENCODER ===
            //reset button for encoders if needed
            if (resetEncoderControl) {
                bot.d2EncoderReset();
            }

            // === TELEMETRY ===
            telemetry.addData("Current Extend Pos: ", bot.getExtendPos());
            telemetry.addData("Current Pivot Pos: ", bot.getPivotArmPos());
            telemetry.addData("Current Left Lift Pos: ", bot.getLeftLiftPos());
            telemetry.addData("Current Right Lift Pos: ", bot.getRightLiftPos());
            telemetry.addData("Left Front Power: ", frontLeftPower);
            telemetry.addData("Left Back Power: ", backLeftPower);
            telemetry.addData("Right Front Power", frontRightPower);
            telemetry.addData("Right Back Power", backRightPower);
            telemetry.update();

        }
    }
}