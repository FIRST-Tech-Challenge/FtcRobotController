package org.firstinspires.ftc.teamcode.Qualifier_2.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Qualifier_2.Components.Accesories.RingDepositor;
import org.firstinspires.ftc.teamcode.Qualifier_2.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Qualifier_2.Robot;
import org.firstinspires.ftc.teamcode.Qualifier_2.Components.BasicChassis;

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
 * @since   2020-November-11
 *
 */


@TeleOp(name = "Teleop ")
//@Disabled
public class Teleop extends LinearOpMode {

    // new version of runopmode that supports inplaceturn slowmode and can toggle slowmode on and off with one button - tested by aiden jonathan ma
    public void runOpMode() {

        telemetry.addData("Status", "Before new Robot");
        telemetry.update();
        Robot robot=new Robot(this, BasicChassis.ChassisType.ENCODER);
        telemetry.addData("Status", "Done with new Robot");
        telemetry.update();

        double magnitude;
        double angleInRadian;
        double angleInDegree;
        boolean isSlow = false;
        boolean currSlow = false;
        boolean slowMode = false;
        boolean moveServo = true;
        boolean servoIsMoved = true;
        boolean ring_clamp_is_up = true;
        boolean move_ring_clamp = true;
        boolean wobble_goal_servo_is_up = true;
        boolean move_wobble_goal_servo = true;
        WobbleGoal.Position currentWobbleGoalPosition = WobbleGoal.Position.REST;
        RingDepositor.Position currentRingDepositorPosition = RingDepositor.Position.REST;

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
            boolean move_wobble_goal_arm = gamepad1.right_bumper;
            boolean smart_depo = gamepad1.left_bumper;
//            float start_intake = gamepad1.right_trigger;
//            float stop_intake = gamepad1.left_trigger;
            boolean wobble_goal_servo = gamepad1.x;
            boolean slow = gamepad1.a;
            boolean b_button = gamepad1.b;
            boolean y_button2 = gamepad2.y;
            boolean b_button2 = gamepad2.b;
            boolean a_button2 = gamepad2.a;
            boolean servo = gamepad2.x;
            float shooter = gamepad2.right_trigger;

            angleInRadian = Math.atan2(left_stick_y, left_stick_x);
            angleInDegree = Math.toDegrees(angleInRadian);

            /**Shooter**/
//            if (servo) {
//                moveServo = true;
//
//                if (servoIsMoved) {
//                    servoIsMoved = false;
//                } else if (servoIsMoved == false) {
//                    servoIsMoved = true;
//                }
//            } else {
//                moveServo = false;
//            }
            if (servo){
                telemetry.addData("Servo", " SERVO Forward and Backward");
                telemetry.update();
                robot.moveServo(false);
                robot.moveServo(true);
            }

//            if (moveServo) {
//                if (servoIsMoved) {
//                    telemetry.addData("Servo", " SERVO FORTH x button");
//                    telemetry.update();
//                    robot.moveServo(true);
//                } else if (servoIsMoved == false) {
//                    telemetry.addData("Servo", " SERVO BACK x button");
//                    telemetry.update();
//                    robot.moveServo(false);
//                }
//            }

            if (shooter != 0) {
                robot.shootGoalTeleop(999999999, 100);
            } else {
                robot.shootGoalTeleop(0, 0);
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
                    nextWobbleGoalPosition = robot.wobbleGoalGoToPosition(WobbleGoal.Position.GRAB);
                } else if (currentWobbleGoalPosition == WobbleGoal.Position.GRAB) {
                    nextWobbleGoalPosition = robot.wobbleGoalGoToPosition(WobbleGoal.Position.RAISE);
                } else if (currentWobbleGoalPosition == WobbleGoal.Position.RAISE) {
                    nextWobbleGoalPosition = robot.wobbleGoalGoToPosition(WobbleGoal.Position.RELEASE);
                } else if (currentWobbleGoalPosition == WobbleGoal.Position.RELEASE) {
                    nextWobbleGoalPosition = robot.wobbleGoalGoToPosition(WobbleGoal.Position.GRAB);
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

            if (wobble_goal_servo) {
                move_wobble_goal_servo = true;

                if (wobble_goal_servo_is_up) {
                    wobble_goal_servo_is_up = false;
                } else if (!wobble_goal_servo_is_up) {
                    wobble_goal_servo_is_up = true;
                }
            } else {
                move_wobble_goal_servo = false;
            }

            if (move_wobble_goal_servo) {
                if (wobble_goal_servo_is_up) {
                    telemetry.addData("Wobble Goal Servo", " Wobble Goal UP y_button");
                    telemetry.update();
                    robot.moveWobbleGoalServo(true);
                } else if (!wobble_goal_servo_is_up) {
                    telemetry.addData("Wobble Goal Servo", " Wobble Goal DOWN y_button");
                    telemetry.update();
                    robot.moveWobbleGoalServo(false);
                }
            }

            // ring depositor
            if (smart_depo){
                robot.ringDepositorSmartDeposit();
            }

            //intake
//            if(start_intake == 1.00){
//                robot.startIntake();
//            } else if (stop_intake == 1.00){
//                robot.stopIntake();
//            }

        }
        idle();
    }
}