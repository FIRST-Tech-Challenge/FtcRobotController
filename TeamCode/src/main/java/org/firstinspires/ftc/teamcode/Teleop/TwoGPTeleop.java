package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.OdometryChassis;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;

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
 * @since   2021-Jan-13
 *
 */


@TeleOp(name = "TwoGPTeleop ")
//@Disabled

public class TwoGPTeleop extends LinearOpMode {
        public void runOpMode() {
            telemetry.addData("Status", "Before new Robot");
            telemetry.update();
            Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, false ,true);
            telemetry.addData("Status", "Done with new Robot");
            telemetry.update();
            //robot.navigateTeleOp();
            double magnitude;
            double angleInRadian;
            double angleInDegree;
            boolean slowMode = false;
            boolean wobble_goal_servo_is_up = true;
            boolean move_wobble_goal_servo = true;
            int leftStickPos = 0;
            int rightStickPos = 1;
            robot.openWobbleGoalClaw();
            WobbleGoal.Position currentWobbleGoalPosition = WobbleGoal.Position.REST;
            robot.moveLeftStick(0);
            robot.moveRightStick(1);
            double yShootingPosition = 0;
            double xShootingPosition = 0;
            double angleShootingPosition = 0;
            int goingToPosition = 0;

            telemetry.addData("Status", "Ready to go");
            telemetry.update();

            //Aiden - during competition day robot disconnected so we are trying this code
            while (!opModeIsActive() && !isStopRequested()) {
                telemetry.addData("status", "waiting for start command...");
                telemetry.update();
            }


            while (!isStopRequested()) {
                float left_stick_y = -gamepad1.left_stick_y;
                float left_stick_x = -gamepad1.left_stick_x;
                float right_stick_x = -gamepad1.right_stick_x;
                boolean move_wobble_goal_arm = gamepad2.b;
                boolean start_transfer_sys = gamepad1.right_bumper;
                float shooter = gamepad2.right_trigger;
                boolean odo_powershots = gamepad1.b;
                boolean shooter_servo = gamepad1.x;
                boolean wobble_goal_servo = gamepad2.y;
                boolean quick_reverse = gamepad1.a;
                boolean move_sticks_down = gamepad2.dpad_up;
                boolean move_sticks_up = gamepad2.dpad_down;
                boolean save_Shooting_Position = gamepad2.a;
                boolean auto_powershots = gamepad2.x;
                float goToShootingPosition = gamepad2.left_trigger;
                boolean moveLeftStick = gamepad2.dpad_left;
                boolean moveRightStick = gamepad2.dpad_right;
                /*OdometryChassis.vuforia_on = gamepad1.y;
                if(gamepad1.y){
                    robot.stopAllMotors();
                    continue;
                }*/


                if(!Robot.isCorgi){
                    angleInRadian = Math.atan2(left_stick_y*-1, left_stick_x*2);
                }
                else{
                    angleInRadian = Math.atan2(left_stick_y, left_stick_x*-2);
                }
                angleInDegree = Math.toDegrees(angleInRadian);
                /**Powershots**/
                if(odo_powershots){
                        //robot.setPosition(0,0,0);
                        goingToPosition=1;
                        robot.setVelocity(1385,3000);
                        robot.goToPosition(5,-28.0 ,0,0.7);
                        robot.shootThreePowerShot();
                        //robot.goToPosition(40,-40,-88,0.7);
                    continue;
                }
                else if(goingToPosition==1&&!odo_powershots&&goToShootingPosition!=1){
                    robot.stopAllMotors();
                    goingToPosition=0;
                }
                if (goToShootingPosition==1){
                    //robot.shootGoalTeleop(1000);
                    goingToPosition=1;
//                    robot.goToPosition(yShootingPosition, xShootingPosition, angleShootingPosition, 1.0);
                    robot.turnInPlace(angleShootingPosition,1.0);
                    /*if(robot.goToPositionTeleop(yShootingPosition, xShootingPosition, , 0.8)){
                        robot.turnInPlace(angleShootingPosition,0.8);
                    }*/
                    //continue;

                }
                /*else if(goingToPosition==1&&goToShootingPosition!=1&&!odo_powershots){
                    robot.stopAllMotors();
                    goingToPosition=0;
                }*/

                /**Powershots**/
                if(auto_powershots){
                    robot.setPosition(0,0,0);
                    robot.goToPosition(6.127,-33.15,0,0.9);
                    robot.shootThreePowerShot();
                }

                /**Sticks**/

                if (move_sticks_down) {
                    robot.moveLeftStick(0);
                    robot.moveRightStick(1);
                }

                if (move_sticks_up){
                    robot.moveLeftStick(1);
                    robot.moveRightStick(0.0);
                }

                if(moveLeftStick){
                    if(leftStickPos==0){
                        robot.moveLeftStick(1);
                        leftStickPos = 1;
                    }
                    else if(leftStickPos==1){
                        robot.moveLeftStick(0);
                        leftStickPos = 0;
                    }
                    sleep(250);
                }

                if(moveRightStick){
                    if(rightStickPos==1){
                        robot.moveRightStick(0);
                        rightStickPos = 0;
                    }
                    else if(rightStickPos==0){
                        robot.moveRightStick(1);
                        rightStickPos = 1;
                    }
                    sleep(250);
                }

                /**Shooter**/
                if (shooter_servo){
                    robot.moveServo(false);
                    robot.moveServo(true);
                }

                if (shooter != 0) {
                    robot.shootGoalTeleop(1000);
                } else {
                    robot.stopShooter();
                }

                magnitude = Math.sqrt(Math.pow(left_stick_x, 2) + Math.sqrt(Math.pow(left_stick_y, 2)));

                robot.moveMultidirectional(magnitude, angleInDegree, (float)(right_stick_x*0.6), slowMode); // It is 0.95, because the robot DCs at full power.
                WobbleGoal.Position nextWobbleGoalPosition = WobbleGoal.Position.GRAB;
                // wobble goal movements
                if (move_wobble_goal_arm){
                    if (currentWobbleGoalPosition == WobbleGoal.Position.REST){
                        nextWobbleGoalPosition = robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
                    } else if (currentWobbleGoalPosition == WobbleGoal.Position.GRAB) {
                        nextWobbleGoalPosition = robot.moveWobbleGoalToPosition(WobbleGoal.Position.DRIVETOWALL);
                    } else if (currentWobbleGoalPosition == WobbleGoal.Position.DRIVETOWALL) {
                        nextWobbleGoalPosition = robot.moveWobbleGoalToPosition(WobbleGoal.Position.RAISE);
                    }
                    else if(currentWobbleGoalPosition == WobbleGoal.Position.RAISE){
                        nextWobbleGoalPosition = robot.moveWobbleGoalToPosition(WobbleGoal.Position.GRAB);
                    }
                    // added by Aiden; must have this otherwise if you hold onto the button multiple
                    // actions/movements will be executed by mistake
                   sleep(200);
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
                        robot.closeWobbleGoalClaw();
                    } else if (!wobble_goal_servo_is_up) {
                        robot.openWobbleGoalClaw();
                    }
                }

                // transfer system
                if(start_transfer_sys){
                    robot.startIntake();
                    robot.startTransfer();
                } else if (!start_transfer_sys&&!quick_reverse){
                    robot.stopIntake();
                    robot.stopTransfer();
                }

                // quick reverse
                if (quick_reverse){
                    robot.reverseIntake();
                    robot.reverseTransfer();
                    sleep(250);
                    robot.startIntake();
                    robot.startTransfer();
                }else if(!quick_reverse&&!start_transfer_sys){
                    robot.stopIntake();
                    robot.stopTransfer();
                }



                if (save_Shooting_Position){
                    yShootingPosition = robot.track()[0];
                    xShootingPosition = robot.track()[1];
                    angleShootingPosition = robot.track()[2];
                }


            }
            idle();
        }
    }