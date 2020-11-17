package org.firstinspires.ftc.teamcode.Qualifier_1.Teleop;

import android.media.FaceDetector;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.RingDepositor;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Before new RObot");
        telemetry.update();
        Robot robot = new Robot(this);
        telemetry.addData("Status", "Done with new Robot");
        telemetry.update();

        double magnitude;
        double angleInRadian;
        double angleInDegree;
        boolean slowMode = false;
        boolean moveServo = true;
        boolean servoIsMoved = true;
        WobbleGoal.Position currentWobbleGoalPosition = WobbleGoal.Position.REST;
        RingDepositor.Position currentRingDepositorPosition = RingDepositor.Position.REST;

        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        // commented by Victor
        // robot.initChassis(this);

        //Aiden - during competition day robot disconnected so we are trying this code
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (!isStopRequested()) {

            float left_stick_y = -gamepad1.left_stick_y;
            float left_stick_x = -gamepad1.left_stick_x;
            float right_stick_x = -gamepad1.right_stick_x;
            boolean move_wobble_goal_arm = gamepad1.right_bumper;
            boolean move_ring_depositor = gamepad1.left_bumper;
//            float start_intake = gamepad1.right_trigger;
//            float stop_intake = gamepad1.left_trigger;
            boolean x_button = gamepad1.x;
            boolean a_button = gamepad1.a;
            boolean ring_clamp_true = gamepad1.b;
            boolean ring_clamp_false = gamepad1.y;
            boolean startingPosition = gamepad2.dpad_up;
            boolean grabbingPosition = gamepad2.dpad_right;
            boolean liftingPosition = gamepad2.dpad_down;
            boolean droppingPosition = gamepad2.dpad_left;
            boolean y_button2 = gamepad2.y;
            boolean b_button2 = gamepad2.b;
            boolean a_button2 = gamepad2.a;
            boolean servo = gamepad2.x;
            float shooter = gamepad2.right_trigger;

            angleInRadian = Math.atan2(left_stick_y, left_stick_x);
            angleInDegree = Math.toDegrees(angleInRadian);

            /**Shooter**/
            if (servo) {
                moveServo = true;

                if (servoIsMoved) {
                    servoIsMoved = false;
                } else if (servoIsMoved == false) {
                    servoIsMoved = true;
                }
            } else {
                moveServo = false;
            }

            if (moveServo) {
                if (servoIsMoved) {
                    telemetry.addData("Servo", " SERVO FORTH x button");
                    telemetry.update();
                    robot.moveServo(true);
                } else if (servoIsMoved == false) {
                    telemetry.addData("Servo", " SERVO BACK x button");
                    telemetry.update();
                    robot.moveServo(false);
                }
            }
            // as of nov 14, 2020, the shooter crashes the program
//            if (shooter != 0) {
//                robot.shootGoalTeleop(999999999, 100);
//            } else {
//                robot.shootGoalTeleop(0, 0);
//            }

            /**Speed Mode**/
            if (a_button) { //click a to turn on slowmode
                slowMode = true;
            }
            if (x_button) { //click x to turn off slow mode
                slowMode = false;
            }

            if (slowMode) {
                if (left_stick_x == 0 && left_stick_y == 0) {
                    magnitude = 0;
                } else {
                    magnitude = 0.3;
                }

            } else {
                magnitude = Math.sqrt(Math.pow(left_stick_x, 2) + Math.sqrt(Math.pow(left_stick_y, 2)));
            }
            robot.multidirectionalMove(magnitude, angleInDegree, right_stick_x);

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
                    nextWobbleGoalPosition = robot.wobbleGoalGoToPosition(WobbleGoal.Position.REST);
                } else {
                    telemetry.addData("Wobble Goal", "u have made a STUPID MISTAKE");
                    telemetry.update();
                    sleep(2000);
                }
                // added by Aiden; must have this otherwise if you hold onto the button multiple
                // actions/movements will be executed by mistake
                sleep(1000);
                currentWobbleGoalPosition = nextWobbleGoalPosition;
            }

            telemetry.addData("Wobble Goal Direct", startingPosition + ", " + grabbingPosition + ", " + liftingPosition + ", " + droppingPosition);
            telemetry.update();
//            sleep(2000);
            if (startingPosition == true) {
                robot.wobbleGoalStartingPosition();
            } else if (grabbingPosition == true) {
                robot.wobbleGoalGrabbingPosition();
            } else if (liftingPosition == true) {
                robot.wobbleGoalLiftingPosition();
            } else if (droppingPosition == true) {
                robot.wobbleGoalDroppingPosition();
            }


            // ring depositor
            if (move_ring_depositor){
                if (currentRingDepositorPosition == RingDepositor.Position.REST){
                    robot.ringDepositorGoToPosition(RingDepositor.Position.LOWGOAL);
                    currentRingDepositorPosition = RingDepositor.Position.LOWGOAL;
                } else if (currentRingDepositorPosition == RingDepositor.Position.LOWGOAL) {
                    robot.ringDepositorGoToPosition(RingDepositor.Position.REST);
                    currentRingDepositorPosition = RingDepositor.Position.REST;
                } else {
                    telemetry.addData("Ring Depositor: ", "u have made a STUPID MISTAKE");
                    telemetry.update();
                    sleep(2000);
                }
            }
//            if (move_ring_depositor_frwd == 1.00){
//                robot.ringDepositorClockwise();
//            } else if (move_ring_depositor_bckwrd == 1.00){
//                robot.ringDepositorCounterClockwise();
//            } else {
//                robot.stopRingDepositor();
//            }

            if (ring_clamp_true){
                robot.moveRingClamp(true);
            }
            if (ring_clamp_false){
                robot.moveRingClamp(false);
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