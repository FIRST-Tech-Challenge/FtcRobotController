package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotUtilities.MovementVars;
import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;

import static java.lang.Math.toRadians;

/**
 * Created by Ethan on 12/2/2016.
 */

//@TeleOp(name="Omni: TeleOpTesty", group ="TeleOp")
public class OmniTeleTest extends OmniAutoXYBase {

    public OmniTeleTest() {
        msStuckDetectInit = 10000;
    }

    public HardwareOmnibot robot = new HardwareOmnibot();

//    @Override
    public void initRobot() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        robot.init(hardwareMap);
        robot.setInputShaping(false);
        robot.resetEncoders();

        // Lets try run without encoders.
        robot.disableDriveEncoders();
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        //give MyPosition our current positions so that it saves the last positions of the wheels
        //this means we won't teleport when we start the match. Just in case, run this twice
        for(int i = 0; i < 2 ; i ++){
            robot.resetReads();
            MyPosition.initialize(robot.getLeftEncoderWheelPosition(),
                    robot.getRightEncoderWheelPosition(),
                    robot.getStrafeEncoderWheelPosition());
        }
        MyPosition.setPosition(0, 0, Math.toRadians(0));
    }

    private double driverAngle = 0.0;
    private final double MAX_SPEED = 1.0;
    private final double MAX_SPIN = 1.0;
    private double speedMultiplier = MAX_SPEED;
    private double spinMultiplier = MAX_SPIN;
    private boolean aHeld = false;
    private boolean bHeld = false;
    private boolean yHeld = false;
    private boolean xHeld = false;
    private boolean upHeld = false;
    private boolean downHeld = false;
    private boolean rightHeld = false;
    private boolean leftHeld = false;
    private boolean rightBumperHeld = false;
    private boolean leftBumperHeld = false;
    private boolean a2Held = false;
    private boolean b2Held = false;
    private boolean y2Held = false;
    private boolean x2Held = false;
    private boolean up2Held = false;
    private boolean down2Held = false;
    private boolean aPressed;
    private boolean bPressed;
    private boolean yPressed;
    private boolean xPressed;
    private boolean upPressed;
    private boolean downPressed;
    private boolean rightPressed;
    private boolean leftPressed;
    private boolean rightBumperPressed;
    private boolean leftBumperPressed;
    private boolean a2Pressed;
    private boolean b2Pressed;
    private boolean y2Pressed;
    private boolean x2Pressed;
    private boolean up2Pressed;
    private boolean down2Pressed;
    private double yPower;
    private double xPower;
    private double spin;
    private double gyroAngle;
    private double liftPower;
    private double extendPower;
    private double collectPower;
    private boolean reverse = false;
    private boolean spinning = false;
    private boolean doneExtending = true;
    private boolean clawOpen = true;
    private boolean clawFront = true;
    private double leftTof;
    private double rightTof;
    private double backLeftTof;
    private double backRightTof;
    private double backTof;
    private double sideDistanceTarget = 30;
    private double backDistanceTarget = 30;
    private double clawPosition = 0;
    private double clawdricopterPosition = 0;
    private boolean gotoHome = false;
    private boolean gotoPosition1 = false;
    private boolean gotoPosition2 = false;
    private boolean gotoPosition3 = false;
    private boolean rotatePosition1 = false;
    private boolean rotatePosition2 = false;
    private ElapsedTime loopTime = new ElapsedTime();
    private double oneLoop;
    private double rotateMin = 0.0;
    private double strafeMin = 0.0;
    private double forwardMin = 0.0;

//    @Override
//    public void start()
//    {
//    }

//    @Override
//    public void loop() {
    @Override
    public void runOpMode() {
        initRobot();
        setupRobotParameters(4.0, 19.2);
        robot.disableDriveEncoders();
        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();
        robot.resetReads();
        MyPosition.giveMePositions(robot.getLeftEncoderWheelPosition(),
                robot.getRightEncoderWheelPosition(),
                robot.getStrafeEncoderWheelPosition());
        MyPosition.setPosition(0, 0, Math.toRadians(0.0));

        while (opModeIsActive()) {
            loopTime.reset();
            //left joystick is for moving
            //right joystick is for rotation
            gyroAngle = robot.readIMU();

            yPower = -HardwareOmnibot.cleanMotionValues(gamepad1.left_stick_y);
            xPower = HardwareOmnibot.cleanMotionValues(gamepad1.left_stick_x);
            spin = -HardwareOmnibot.cleanMotionValues(gamepad1.right_stick_x);
            aPressed = gamepad1.a;
            bPressed = gamepad1.b;
            yPressed = gamepad1.y;
            xPressed = gamepad1.x;
            upPressed = gamepad1.dpad_up;
            downPressed = gamepad1.dpad_down;
            rightPressed = gamepad1.dpad_right;
            leftPressed = gamepad1.dpad_left;
            rightBumperPressed = gamepad1.right_bumper;
            leftBumperPressed = gamepad1.left_bumper;
            a2Pressed = gamepad2.a;
            b2Pressed = gamepad2.b;
            y2Pressed = gamepad2.y;
            x2Pressed = gamepad2.x;
            up2Pressed = gamepad2.dpad_up;
            down2Pressed = gamepad2.dpad_down;

            // Allow the robot to read sensors again
            robot.resetReads();
            MyPosition.giveMePositions(robot.getLeftEncoderWheelPosition(),
                    robot.getRightEncoderWheelPosition(),
                    robot.getStrafeEncoderWheelPosition());

            if (!xHeld && xPressed) {
                xHeld = true;
                gotoPosition1 = true;
                robot.driveToXY(250, 0, 0, 0.3, false, false);
                robot.resetReads();
            } else if (!xPressed) {
                xHeld = false;
            }

            if (!rightBumperHeld && rightBumperPressed) {
                // Lets get forward minimum determined
                rotateMin += 0.01;
                robot.frontLeft.setPower(rotateMin);
                robot.frontRight.setPower(rotateMin);
                robot.rearLeft.setPower(rotateMin);
                robot.rearRight.setPower(rotateMin);
                rightBumperHeld = true;
            } else if (!rightBumperPressed) {
                rightBumperHeld = false;
            }

            if (!leftBumperHeld && leftBumperPressed) {
                rotateMin -= 0.01;
                robot.frontLeft.setPower(rotateMin);
                robot.frontRight.setPower(rotateMin);
                robot.rearLeft.setPower(rotateMin);
                robot.rearRight.setPower(rotateMin);
                leftBumperHeld = true;
            } else if (!leftBumperPressed) {
                leftBumperHeld = false;
            }

            if (!aHeld && aPressed) {
                robot.fingersDown();
                aHeld = true;
            } else if (!aPressed) {
                aHeld = false;
            }

            if (!bHeld && bPressed) {
                bHeld = true;
                telemetry.addLine("STOPPING EVERYTHING!!!");
                rotateMin = 0.0;
                strafeMin = 0.0;
                forwardMin = 0.0;
                gotoHome = false;
                gotoPosition1 = false;
                gotoPosition2 = false;
                gotoPosition3 = false;
                robot.setAllDriveZero();
            } else if (!bPressed) {
                bHeld = false;
            }

            if (!yHeld && yPressed) {
                gotoPosition2 = true;
                robot.driveToXY(0, 250, 0, 0.3, false, false);
                robot.resetReads();
                yHeld = true;
            } else if (!yPressed) {
                yHeld = false;
            }

            if (!upHeld && upPressed) {
                // Lets get forward minimum determined
                forwardMin += 0.01;
                robot.frontLeft.setPower(forwardMin);
                robot.frontRight.setPower(-forwardMin);
                robot.rearLeft.setPower(forwardMin);
                robot.rearRight.setPower(-forwardMin);
                upHeld = true;
            } else if (!upPressed) {
                upHeld = false;
            }

            if (!downHeld && downPressed) {
                // Lets get forward minimum determined
                forwardMin -= 0.01;
                robot.frontLeft.setPower(forwardMin);
                robot.frontRight.setPower(-forwardMin);
                robot.rearLeft.setPower(forwardMin);
                robot.rearRight.setPower(-forwardMin);
                downHeld = true;
            } else if (!downPressed) {
                downHeld = false;
            }

            if (!rightHeld && rightPressed) {
                // Lets get stafe minimum determined
                strafeMin += 0.01;
                robot.frontLeft.setPower(strafeMin);
                robot.frontRight.setPower(strafeMin);
                robot.rearLeft.setPower(-strafeMin);
                robot.rearRight.setPower(-strafeMin);
                rightHeld = true;
            } else if (!rightPressed) {
                rightHeld = false;
            }

            if (!leftHeld && leftPressed) {
                // Lets get stafe minimum determined
                strafeMin -= 0.01;
                robot.frontLeft.setPower(strafeMin);
                robot.frontRight.setPower(strafeMin);
                robot.rearLeft.setPower(-strafeMin);
                robot.rearRight.setPower(-strafeMin);
                leftHeld = true;
            } else if (!leftPressed) {
                leftHeld = false;
            }
            if(gotoHome) {
                if(robot.driveToXY(0, 0, 0, 0.3, false, false)) {
                    telemetry.addLine("Drive to Home TRUE");
                    robot.setAllDriveZero();
                    gotoHome = false;
                } else {
                    telemetry.addLine("Drive to Home FALSE");
                }
            }
            if(gotoPosition1) {
                if(robot.driveToXY(250, 0, 0, 0.3, false, false)) {
                    telemetry.addLine("Drive to Position 1 TRUE");
                    robot.setAllDriveZero();
                    gotoPosition1 = false;
                } else {
                    telemetry.addLine("Drive to Position 1 FALSE");
                }
            }
            if(gotoPosition2) {
                if(robot.driveToXY(0, 250, 0, 0.3, false, false)) {
                    telemetry.addLine("Drive to Position 2 TRUE");
                    robot.setAllDriveZero();
                    gotoPosition2 = false;
                } else {
                    telemetry.addLine("Drive to Position 2 FALSE");
                }
            }
            if(gotoPosition3) {
                if(robot.driveToXY(100, 100, 0, 0.3, false, false)) {
                    telemetry.addLine("Drive to Position 3 TRUE");
                    robot.setAllDriveZero();
                    gotoPosition3 = false;
                } else {
                    telemetry.addLine("Drive to Position 3 FALSE");
                }
            }
            if(rotatePosition1) {
                if(robot.rotateToAngle(toRadians(-90.0), false, false)) {
                    telemetry.addLine("Rotate to Position 1 TRUE");
                    robot.setAllDriveZero();
                    rotatePosition1 = false;
                } else {
                    telemetry.addLine("Rotate to Position 1 FALSE");
                }
            }
            if(rotatePosition2) {
                if(robot.rotateToAngle(toRadians(90.0), false, false)) {
                    telemetry.addLine("Rotate to Position 2 TRUE");
                    robot.setAllDriveZero();
                    rotatePosition2 = false;
                } else {
                    telemetry.addLine("Rotate to Position 2 FALSE");
                }
            }
        if(!a2Held && a2Pressed)
        {
            a2Held = true;
        } else if(!a2Pressed) {
            a2Held = false;
        }

        if(!b2Held && b2Pressed)
        {
            robot.clawdricopter.setPosition(HardwareOmnibot.CLAWDRICOPTER_FRONT);
            clawdricopterPosition = HardwareOmnibot.CLAWDRICOPTER_FRONT;
            b2Held = true;
        } else if(!b2Pressed) {
            b2Held = false;
        }

        if(!y2Held && y2Pressed)
        {
            robot.clawdricopter.setPosition(HardwareOmnibot.CLAWDRICOPTER_CAPSTONE);
            clawdricopterPosition = HardwareOmnibot.CLAWDRICOPTER_CAPSTONE;
            y2Held = true;
        } else if(!y2Pressed) {
            y2Held = false;
        }

        if(!x2Held && x2Pressed)
        {
            robot.clawdricopter.setPosition(HardwareOmnibot.CLAWDRICOPTER_BACK);
            clawdricopterPosition = HardwareOmnibot.CLAWDRICOPTER_BACK;
            x2Held = true;
        } else if(!x2Pressed) {
            x2Held = false;
        }

        if(!up2Held && up2Pressed)
        {
            clawdricopterPosition += 0.01;
            robot.scissorLatch.setPosition(clawdricopterPosition);
            up2Held = true;
        } else if (!up2Pressed) {
			up2Held = false;
		}

        if(!down2Held && down2Pressed)
        {
            clawdricopterPosition -= 0.01;
            robot.scissorLatch.setPosition(clawdricopterPosition);
            down2Held = true;
        } else if (!down2Pressed) {
			down2Held = false;
		}

//        if(Math.abs(xPower) > 0.1) {
//            robot.manualExtendIntake(xPower);
//        } else {
//            robot.manualExtendIntake(0.0);
//        }

//        if(Math.abs(spin) > 0.1) {
//            robot.manualLift(spin);
//        } else {
//            robot.manualLift(0.0);
//        }
            // If the activity is not performing, it will be idle and return.
//        robot.performLifting();
//        robot.performReleasing();
//        robot.performStowing();


            //telemetry.addData("Left TOF: ", leftTof);
            //telemetry.addData("Right TOF: ", rightTof);
            //telemetry.addData("Back TOF: ", backTof);
            telemetry.addData("Lift Target Height: ", robot.liftTargetHeight.toString());
            telemetry.addData("Lift State: ", robot.liftState);
            telemetry.addData("Release State: ", robot.releaseState);
            telemetry.addData("Stow State: ", robot.stowState);
            telemetry.addData("Y Power: ", yPower);
            telemetry.addData("X Power: ", xPower);
            telemetry.addData("Spin: ", spin);
            telemetry.addData("Offset Angle: ", driverAngle);
            telemetry.addData("Gyro Angle: ", gyroAngle);
            telemetry.addData("Last Angle", robot.lastDriveAngle);
            telemetry.addData("Goto Home", gotoHome);
            telemetry.addData("Goto Position 1", gotoPosition1);
            telemetry.addData("Goto Position 2", gotoPosition2);
            telemetry.addData("Left Encoder: ", robot.getLeftEncoderWheelPosition());
            telemetry.addData("Strafe Encoder: ", robot.getStrafeEncoderWheelPosition());
            telemetry.addData("Right Encoder: ", robot.getRightEncoderWheelPosition());
            telemetry.addData("World X Position: ", MyPosition.worldXPosition);
            telemetry.addData("World Y Position: ", MyPosition.worldYPosition);
            telemetry.addData("World Angle: ", Math.toDegrees(MyPosition.worldAngle_rad));
            telemetry.addData("Movement X: ", MovementVars.movement_x);
            telemetry.addData("Movement Y: ", MovementVars.movement_y);
            telemetry.addData("Movement Turn: ", Math.toDegrees(MovementVars.movement_turn));
            telemetry.addData("Front Left Power: ", robot.frontLeftMotorPower);
            telemetry.addData("Front Right Power: ", robot.frontRightMotorPower);
            telemetry.addData("Rear Left Power: ", robot.rearLeftMotorPower);
            telemetry.addData("Rear Right Power: ", robot.rearRightMotorPower);
            telemetry.addData("Loop time: ", loopTime.milliseconds());
            telemetry.addData("Forward Min: ", forwardMin);
            telemetry.addData("Stafe Min: ", strafeMin);
            telemetry.addData("Rotate Min: ", rotateMin);
            telemetry.addData("Clawdricopter Position: ", clawdricopterPosition);
            updateTelemetry(telemetry);
//            sleep(25);
        }
    }

//    @Override
//    public void stop() {
//        robot.stopGroundEffects();
//    }
}
