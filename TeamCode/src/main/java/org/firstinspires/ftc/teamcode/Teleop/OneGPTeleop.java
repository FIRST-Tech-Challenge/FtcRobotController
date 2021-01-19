package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Teleop w/ drivetrain, shooter(still in testing), and wobblegoal
 *
 * Current button mappings for gamepad 1 = right joystick for movement, left for turning,
 * x for slowmode on, a for slowmode off
 *
 * Current button mappings for gamepad 2 = y for shoot high goal, b for mid goal, a for low,
 * x for moving servo back and forth, right trigger for turning on shooter motor, dpad up and down
 * for wobblegoal
 *
 * @author  Nathan
 * @version 1.0
 * @since   2020-Jan-13
 *
 */


@TeleOp(name = "OneGPTeleop ")
//@Disabled
public class OneGPTeleop extends LinearOpMode {

    // new version of runopmode that supports inplaceturn slowmode and can toggle slowmode on and off with one button - tested by aiden jonathan ma
    public void runOpMode() {

        telemetry.addData("Status", "Before new Robot");
        telemetry.update();
        Robot robot=new Robot(this, BasicChassis.ChassisType.ENCODER, false ,false);
        telemetry.addData("Status", "Done with new Robot");
        telemetry.update();

        double magnitude;
        double angleInRadian;
        double angleInDegree;
        boolean isSlow = false;
        boolean currSlow = false;
        boolean slowMode = false;
        boolean wobble_goal_servo_is_up = true;
        boolean move_wobble_goal_servo = true;
        WobbleGoal.Position currentWobbleGoalPosition = WobbleGoal.Position.REST;

        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        //Aiden - during competition day robot disconnected so we are trying this code
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        robot.teleopStartPosition();

        while (!isStopRequested()) {

            float left_stick_y = -gamepad1.left_stick_y;
            float left_stick_x = -gamepad1.left_stick_x;
            float right_stick_x = -gamepad1.right_stick_x;
            boolean move_wobble_goal_arm = gamepad1.left_bumper;
            boolean start_transfer_sys = gamepad1.right_bumper;
//            boolean wobble_goal_servo = gamepad1.y;
            boolean slow = gamepad1.a;
            boolean shooter_servo = gamepad1.x;
            float shooter = gamepad1.right_trigger;

            angleInRadian = Math.atan2(left_stick_y, left_stick_x);
            angleInDegree = Math.toDegrees(angleInRadian);

            /**Shooter**/
            if (shooter_servo){
                telemetry.addData("Servo", " SERVO Forward and Backward");
                telemetry.update();
                robot.moveServo(false);
                robot.moveServo(true);
            }

            if (shooter != 0) {
                robot.shootGoalTeleop(1000);
            } else {
                robot.stopShooter();
            }

            /**Speed Mode**/
            if (slow) {
                isSlow = true;

                if (currSlow) {
                    currSlow = false;
                } else if (currSlow == false) {
                    currSlow = true;
                }
            } else {
                isSlow = false;
            }

            if (isSlow) {
                if (currSlow) {
                    slowMode = true;
                } else if (currSlow == false) {
                    slowMode = false;
                }
            }

            magnitude = Math.sqrt(Math.pow(left_stick_x, 2) + Math.sqrt(Math.pow(left_stick_y, 2)));

            robot.moveMultidirectional(magnitude, angleInDegree, right_stick_x, slowMode);

            // wobble goal movements
            telemetry.addData("Wobble Goal Toggle", move_wobble_goal_arm + ", " + currentWobbleGoalPosition);
            telemetry.update();
            if (move_wobble_goal_arm){
                WobbleGoal.Position nextWobbleGoalPosition = WobbleGoal.Position.REST;
                if (currentWobbleGoalPosition == WobbleGoal.Position.REST){
                    nextWobbleGoalPosition = robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
                } else if (currentWobbleGoalPosition == WobbleGoal.Position.GRAB) {
                    nextWobbleGoalPosition = robot.moveWobbleGoalToPosition(WobbleGoal.Position.RAISE);
                } else if (currentWobbleGoalPosition == WobbleGoal.Position.RAISE) {
                    nextWobbleGoalPosition = robot.moveWobbleGoalToPosition(WobbleGoal.Position.RELEASE);
                } else if (currentWobbleGoalPosition == WobbleGoal.Position.RELEASE) {
                    nextWobbleGoalPosition = robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
                } else {
                    telemetry.addData("Wobble Goal", "u have made a STUPID MISTAKE");
                    telemetry.update();
                    sleep(500);
                }
                // added by Aiden; must have this otherwise if you hold onto the button multiple
                // actions/movements will be executed by mistake
                sleep(500);
                currentWobbleGoalPosition = nextWobbleGoalPosition;
            }

//            if (wobble_goal_servo) {
//                move_wobble_goal_servo = true;
//
//                if (wobble_goal_servo_is_up) {
//                    wobble_goal_servo_is_up = false;
//                } else if (!wobble_goal_servo_is_up) {
//                    wobble_goal_servo_is_up = true;
//                }
//            } else {
//                move_wobble_goal_servo = false;
//            }
//
//            if (move_wobble_goal_servo) {
//                if (wobble_goal_servo_is_up) {
//                    telemetry.addData("Wobble Goal Servo", " Wobble Goal UP y_button");
//                    telemetry.update();
//                    robot.moveWobbleGoalServo(true);
//                } else if (!wobble_goal_servo_is_up) {
//                    telemetry.addData("Wobble Goal Servo", " Wobble Goal DOWN y_button");
//                    telemetry.update();
//                    robot.moveWobbleGoalServo(false);
//                }
//            }

            //transfer system
            if(start_transfer_sys){
                robot.startIntake();
                robot.startTransfer();
            } else if (!start_transfer_sys){
                robot.stopIntake();
                robot.stopTransfer();
            }

        }
        idle();
    }
}