package org.firstinspires.ftc.teampractice;

import static org.firstinspires.ftc.teampractice.Robot.grabberPosition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Gamepad", group = "Linear Opmode")
public class GamepadOpMode extends OpModeBase {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean listeningForButtonPressRight = false, holdPosition = false;
        ElapsedTime rightButtonTimer = new ElapsedTime();
        int highPolePos = 1950, mediumPolePos = 1320, lowPolePos = 760, junctionPos = 200, cone5Pos = 250;
        int currentTarget = 0;
        int conesOnStack = 5;
        boolean rightBumperPressedOnce = false;
        robot.init(hardwareMap, false);

        //region telemetry setup
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("lift", () -> String.format("pos: %d, cur: %.2fA", robot.armMotor.getCurrentPosition(), robot.armMotor.getCurrent(CurrentUnit.AMPS)));
        telemetry.addData("Motor", () -> String.format("LF: %.2f, RF: %.2f, LR: %.2f, RR: %.2f", robot.leftFront.getVelocity(), robot.rightFront.getVelocity(), robot.leftRear.getVelocity(), robot.rightRear.getVelocity()));
        telemetry.update();
        //endregion

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //region arm control
            int currentArmPosition = robot.armMotor.getCurrentPosition();

            if (gamepad1.left_trigger > 0d) { // raise lift
                robot.armMotor.setTargetPosition(highPolePos);
                robot.setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armMotor.setPower(gamepad1.left_trigger);
                currentTarget = currentArmPosition;
            } else if (gamepad1.right_trigger > 0d && // lower lift
                    robot.limitSwitch.getState() == true) { // when limit sensor not pressed
                double powerScale = currentArmPosition > 800 ? 1d : 0.4d;
                robot.setArmMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.armMotor.setPower(-gamepad1.right_trigger * powerScale);
                currentTarget = currentArmPosition;
                holdPosition = false;
            } else { // stop lift
                if (holdPosition == false && robot.limitSwitch.getState() == false) { // limit sensor pressed
                    robot.setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    currentTarget = 0;
                } else {
                    robot.armMotor.setTargetPosition(currentTarget);
                    robot.setArmMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    robot.armMotor.setPower(currentArmPosition > 100 ? 0.8d : 0.3d);
                }
            }

            if (gamepad1.left_bumper) {
                currentTarget = highPolePos;
                holdPosition = true;
                rightBumperPressedOnce = false;
            } else if (gamepad1.right_bumper && !rb_pressed && robot.limitSwitch.getState()) { // limit sensor not pressed
                listeningForButtonPressRight = true;
                if (!rightBumperPressedOnce) {
                    rightBumperPressedOnce = true;
                    currentTarget -= 200;
                    currentTarget = Math.max(currentTarget, -50);
                    holdPosition = true;
                }
                else {
                    currentTarget = -10;
                    holdPosition = false;
                }
            }
                    if (gamepad1.y) {
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        currentTarget = mediumPolePos;
                        rightBumperPressedOnce = false;
                        holdPosition = true;
                        robot.armMotor.setTargetPosition(currentTarget);
                        robot.armMotor.setPower(0.8d);
                    } else if (gamepad1.x && !listeningForButtonPressRight) {
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        currentTarget = lowPolePos;
                        rightBumperPressedOnce = false;
                        holdPosition = true;
                        robot.armMotor.setTargetPosition(currentTarget);
                        robot.armMotor.setPower(0.8d);
                    } else if (gamepad1.b) {
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        currentTarget = junctionPos;
                        rightBumperPressedOnce = false;
                        holdPosition = true;
                        robot.armMotor.setTargetPosition(currentTarget);
                        robot.armMotor.setPower(0.8d);
                    }

            if (listeningForButtonPressRight) {
                if (gamepad1.x) {
                    currentTarget = cone5Pos - (conesOnStack * 40);
                    if (conesOnStack > 0) {
                        conesOnStack--;
                    }
                    robot.armMotor.setTargetPosition(currentTarget);
                    robot.armMotor.setPower(0.8d);
                }
                if (rightButtonTimer.milliseconds() > 500) {
                    listeningForButtonPressRight = false;
                }
            }

            //endregion


            //region grabber servo control
            if (gamepad1.a && !a_pressed) {
                // close grabber
                robot.setGrabber(grabberPosition == 1 ? 0 : 1);
            }
            //endregion

            //region drivetrain control
            double drive = Math.tan((-gamepad1.left_stick_y - gamepad1.right_stick_y) * Math.tan(0.7d));
            double turn = Math.atan(gamepad1.left_stick_x * Math.tan(0.9d));
            double side = Math.atan(gamepad1.right_stick_x * Math.tan(0.9d));

            double pLeftFront = Range.clip(drive + turn + side, -1.0d, 1.0d);
            double pLeftRear = Range.clip(drive + turn - side, -1.0d, 1.0d);
            double pRightFront = Range.clip(drive - turn - side, -1.0d, 1.0d);
            double pRightRear = Range.clip(drive - turn + side, -1.0d, 1.0d);

            if (gamepad1.dpad_up) {
                pLeftFront = pLeftRear = pRightFront = pRightRear = 1;
            }
            else if (gamepad1.dpad_down) {
                pLeftFront = pLeftRear = pRightFront = pRightRear = -1;
            }
            else if (gamepad1.dpad_left) {
                pLeftFront = pRightRear = -1;
                pLeftRear = pRightFront = 1;
            }
            else if (gamepad1.dpad_right) {
                pLeftFront = pRightRear = 1;
                pLeftRear = pRightFront = -1;
            }

            // Send calculated power to wheels
            robot.setDrivePower(pLeftFront, pLeftRear, pRightFront, pRightRear);
            //endregion

            if (gamepad1.back && !back_pressed) {
                robot.toggleFrontServo();
            }

            gamepadUpdate();
            telemetry.update();
        }
    }
}