package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.WayPoint;
import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;

import static java.lang.Math.atan2;
import static java.lang.Math.toDegrees;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.SHOOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.WOBBLE_ARM_GRABBING;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.WOBBLE_ARM_MAX;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.WOBBLE_ARM_MIN;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.WOBBLE_ARM_RUNNING;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.WOBBLE_ARM_STOWED;

/**
 * Created by 7592 Roarbots.
 */

@TeleOp(name="UltimateGoal: TeleOp", group ="TeleOp")
public class UltimateGoalTeleOp extends OpMode {

    public UltimateGoalRobot robot = new UltimateGoalRobot();
    protected boolean aligning = false;

    @Override
    public void init() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        robot.init(hardwareMap);
//        robot.disableDriveEncoders();
        robot.setShooterFlapPowerShot();
        robot.setInputShaping(true);
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Press X on driver controller to reset encoders.");
        telemetry.addLine("Press O on driver controller to enable Driver Centric Mode.");
        if(gamepad1.cross) {
            robot.forceReset = true;
        }
        if(gamepad1.circle) {
            robot.disableDriverCentric = false;
        }
    }

    private double driverAngle = 0.0;
    private final double MAX_SPEED = 1.0;
    private final double MAX_SPIN = 1.0;
    private double speedMultiplier = MAX_SPEED;
    private double spinMultiplier = MAX_SPIN;
    private boolean crossHeld = false;
    private boolean circleHeld = false;
    private boolean triangleHeld = false;
    private boolean upHeld = false;
    private boolean downHeld = false;
    private boolean leftHeld = false;
    private boolean rightHeld = false;
    private boolean leftBumperHeld = false;
    private boolean rightBumperHeld = false;
    private boolean cross2Held = false;
    private boolean circle2Held = false;
    private boolean triangle2Held = false;
    private boolean square2Held = false;
    private boolean up2Held = false;
    private boolean down2Held = false;
    private boolean left2Held = false;
    private boolean right2Held = false;
    private boolean leftBumper2Held = false;
    private boolean rightBumper2Held = false;
    private boolean crossPressed;
    private boolean circlePressed;
    private boolean trianglePressed;
    private boolean leftPressed;
    private boolean rightPressed;
    private boolean leftBumperPressed;
    private boolean rightBumperPressed;
    private boolean upPressed;
    private boolean downPressed;
    private boolean cross2Pressed;
    private boolean circle2Pressed;
    private boolean triangle2Pressed;
    private boolean square2Pressed;
    private boolean up2Pressed;
    private boolean down2Pressed;
    private boolean left2Pressed;
    private boolean right2Pressed;
    private boolean leftBumper2Pressed;
    private boolean rightBumper2Pressed;
    private boolean fingersUp = true;
    private double rightTriggerPower;
    private double leftTriggerPower;
    private double yPower;
    private double xPower;
    private double spin;
    private ElapsedTime loopTime = new ElapsedTime();
    private boolean runWithEncoders = true;
    private boolean shootHighGoal = true;

    @Override
    public void start() {
        robot.resetEncoders();

        //give MyPosition our current positions so that it saves the last positions of the wheels
        //this means we won't teleport when we start the match. Just in case, run this twice
        for(int i = 0; i < 2 ; i ++) {
            robot.resetReads();
            MyPosition.initialize(robot.getLeftEncoderWheelPosition(),
                    robot.getRightEncoderWheelPosition(),
                    robot.getStrafeEncoderWheelPosition());
        }

        robot.highGoal = new WayPoint(110.10, 153.99, Math.toRadians(87.45), 1.0);
        MyPosition.setPosition(robot.finalAutoPosition.x, robot.finalAutoPosition.y,
                robot.finalAutoPosition.angle);
    }

    @Override
    public void loop() {
        loopTime.reset();
        // Allow the robot to read sensors again
        robot.resetReads();
        MyPosition.giveMePositions(robot.getLeftEncoderWheelPosition(),
                robot.getRightEncoderWheelPosition(),
                robot.getStrafeEncoderWheelPosition());

        //left joystick is for moving
        //right joystick is for rotation
        yPower = -gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;
        leftTriggerPower = gamepad1.left_trigger;
        rightTriggerPower = gamepad1.right_trigger;
        spin = gamepad1.right_stick_x;
        crossPressed = gamepad1.cross;
        circlePressed = gamepad1.circle;
        trianglePressed = gamepad1.triangle;
		rightPressed = gamepad1.dpad_right;
		leftPressed = gamepad1.dpad_left;
		leftBumperPressed = gamepad1.left_bumper;
        rightBumperPressed = gamepad1.right_bumper;
        upPressed = gamepad1.dpad_up;
        downPressed = gamepad1.dpad_down;
        cross2Pressed = gamepad2.cross;
        circle2Pressed = gamepad2.circle;
        triangle2Pressed = gamepad2.triangle;
        square2Pressed = gamepad2.square;
        up2Pressed = gamepad2.dpad_up;
        down2Pressed = gamepad2.dpad_down;
        right2Pressed = gamepad2.dpad_right;
        left2Pressed = gamepad2.dpad_left;
        leftBumper2Pressed = gamepad2.left_bumper;
        rightBumper2Pressed = gamepad2.right_bumper;

        if (gamepad1.square) {
            // The driver presses Square, then uses the left joystick to say what angle the robot
            // is aiming.  This will calculate the values as long as square is pressed, and will
            // not drive the robot using the left stick.  Once square is released, it will use the
            // final calculated angle and drive with the left stick.  Button should be released
            // before stick.  The default behavior of atan2 is 0 to -180 on Y Axis CCW, and 0 to
            // 180 CW.  This code normalizes that to 0 to 360 CCW from the Y Axis
            //robot.resetGyro();
            // This was -90, but needed to be oriented 90 degrees from actual angle.
            driverAngle = toDegrees(atan2(yPower, xPower))- 90.0 - robot.readIMU();
            xPower = 0.0;
            yPower = 0.0;
            spin = 0.0;
        }

		// ********************************************************************
		// DRIVER JOYSTICK
		// ********************************************************************
        if(!circleHeld && circlePressed) {
            if(robot.clawClosed) {
                robot.startClawToggle(true);
            } else {
                robot.startClawToggle(false);
            }
            circleHeld = true;
        } else if(!circlePressed) {
            circleHeld = false;
        }

        if(trianglePressed) {
            if(robot.injectState == UltimateGoalRobot.INJECTING.IDLE) {
                if (shootHighGoal) {
                    UltimateGoalRobot.shootingError.x = MyPosition.worldXPosition - UltimateGoalRobot.highGoal.x;
                    UltimateGoalRobot.shootingError.y = MyPosition.worldYPosition - UltimateGoalRobot.highGoal.y;
                    UltimateGoalRobot.shootingError.angle = MyPosition.worldAngle_rad - UltimateGoalRobot.highGoal.angle;
                }
                robot.startInjecting();
            }
        }
//        if(!triangleHeld && trianglePressed) {
//            if(robot.flapPosition == UltimateGoalRobot.FLAP_POSITION.HIGH_GOAL) {
//                UltimateGoalRobot.shootingError.x = MyPosition.worldXPosition - UltimateGoalRobot.highGoal.x;
//                UltimateGoalRobot.shootingError.y = MyPosition.worldYPosition - UltimateGoalRobot.highGoal.y;
//                UltimateGoalRobot.shootingError.angle = MyPosition.worldAngle_rad - UltimateGoalRobot.highGoal.angle;
//            }
//            robot.startTripleInjecting();
//            triangleHeld = true;
//        } else if(!trianglePressed) {
//            triangleHeld = false;
//        }

        if(!crossHeld && crossPressed) {
            crossHeld = true;
            if(robot.shooterMotorTargetVelocity == SHOOT_VELOCITY) {
                robot.shooterOff();
            } else {
                robot.shooterOnHighGoal();
            }
        } else if(!crossPressed) {
            crossHeld = false;
        }

        // This can be used for shoot alignment.
        if(!rightHeld && rightPressed) {
            if(!aligning) {
                robot.startShotAligning(UltimateGoalRobot.powerShotRight, true);
                aligning = true;
            } else {
                aligning = false;
                robot.stopShotAligning();
            }
            shootHighGoal = false;
            rightHeld = true;
        } else if(!rightPressed) {
            rightHeld = false;
        }

        if(!leftHeld && leftPressed) {
            if(!aligning) {
                robot.startShotAligning(UltimateGoalRobot.powerShotLeft, true);
                aligning = true;
            } else {
                aligning = false;
                robot.stopShotAligning();
            }
            shootHighGoal = false;
            leftHeld = true;
        } else if(!leftPressed) {
            leftHeld = false;
        }

        if(!downHeld && downPressed) {
            if(!aligning) {
                robot.startShotAligning(UltimateGoalRobot.powerShotCenter, true);
                aligning = true;
            } else {
                aligning = false;
                robot.stopShotAligning();
            }
            shootHighGoal = false;
            downHeld = true;
        } else if (!downPressed) {
            downHeld = false;
        }

        if(!upHeld && upPressed) {
            if(!aligning) {
                robot.startShotAligning(UltimateGoalRobot.highGoal, false);
                aligning = true;
            } else {
                aligning = false;
                robot.stopShotAligning();
            }
            shootHighGoal = true;
            upHeld = true;
        } else if (!upPressed) {
            upHeld = false;
        }

        if(!rightBumperHeld && rightBumperPressed) {
            if(robot.intakeMotorPower != 0.0) {
                robot.setIntakeOff();
            } else {
                robot.setIntakeIn();
            }
            rightBumperHeld = true;
        } else if(!rightBumperPressed) {
            rightBumperHeld = false;
        }

        if(!leftBumperHeld && leftBumperPressed) {
            if(robot.intakeMotorPower != 0.0) {
                robot.setIntakeOff();
            } else {
                robot.setIntakeOut();
            }
            leftBumperHeld = true;
        } else if(!leftBumperPressed) {
            leftBumperHeld = false;
        }

        if (rightTriggerPower >= 0.05) {
            if(robot.armPot.getVoltage() > WOBBLE_ARM_MIN) {
                robot.setWobbleMotorPower(-rightTriggerPower);
            } else {robot.setWobbleMotorPower(0.0);}
        } else if (leftTriggerPower >= 0.05) {
            if(robot.armPot.getVoltage() < WOBBLE_ARM_MAX) {
                robot.setWobbleMotorPower(leftTriggerPower);
            } else {robot.setWobbleMotorPower(0.0);}
        } else {
            robot.setWobbleMotorPower(0.0);
        }

		// ********************************************************************
		// OPERATOR JOYSTICK
		// ********************************************************************
		// This was unassigned (fingers up/down)
        if(!square2Held && square2Pressed) {
            // Enable the velocity checks again.
            robot.disableVelocityCheck = false;
            square2Held = true;
        } else if(!square2Pressed) {
            square2Held = false;
        }

        if(!cross2Held && cross2Pressed) {
            robot.startRotatingArm(WOBBLE_ARM_GRABBING);
            cross2Held = true;
        } else if(!cross2Pressed) {
            cross2Held = false;
        }

        if(!circle2Held && circle2Pressed) {
            robot.startRotatingArm(WOBBLE_ARM_RUNNING);
            circle2Held = true;
        } else if(!circle2Pressed) {
            circle2Held = false;
        }

        if(!triangle2Held && triangle2Pressed) {
            robot.startRotatingArm(WOBBLE_ARM_STOWED);
            triangle2Held = true;
        } else if(!triangle2Pressed) {
            triangle2Held = false;
        }

        if(!left2Held && left2Pressed) {
            left2Held = true;
        } else if (!left2Pressed) {
            left2Held = false;
        }

        if(!right2Held && right2Pressed) {
            right2Held = true;
        } else if (!right2Pressed) {
            right2Held = false;
        }

        if(!up2Held && up2Pressed) {
            if(robot.flapPosition == UltimateGoalRobot.FLAP_POSITION.HIGH_GOAL) {
                robot.highGoalOffset -= 0.002;
                robot.setShooterFlapHighGoal();
            } else {
                robot.powerShotOffset -= 0.002;
                robot.setShooterFlapPowerShot();
            }
            up2Held = true;
        } else if (!up2Pressed) {
			up2Held = false;
		}

        if(!down2Held && down2Pressed) {
            if(robot.flapPosition == UltimateGoalRobot.FLAP_POSITION.HIGH_GOAL) {
                robot.highGoalOffset += 0.002;
                robot.setShooterFlapHighGoal();
            } else {
                robot.powerShotOffset += 0.002;
                robot.setShooterFlapPowerShot();
            }
            down2Held = true;
        } else if (!down2Pressed) {
			down2Held = false;
		}

        if(!rightBumper2Held && rightBumper2Pressed) {
            robot.shooterOnPowershot();
           rightBumper2Held = true;
        } else if(!rightBumper2Pressed) {
            rightBumper2Held = false;
        }

        if(!leftBumper2Held && leftBumper2Pressed) {
            robot.shooterOnHighGoal();
            leftBumper2Held = true;
        } else if(!leftBumper2Pressed) {
            leftBumper2Held = false;
        }

        // If the activity is not performing, it will be idle and return.
        performActivities();
        if(robot.shotAlignmentState == UltimateGoalRobot.SHOT_ALIGNMENT_STATE.IDLE) {
            aligning = false;
            robot.drive(speedMultiplier * xPower, speedMultiplier * yPower,
                    spinMultiplier * spin, driverAngle - 90.0, robot.defaultInputShaping);
        }

        telemetry.addData("Shooter Target Velocity: ", robot.shooterMotorTargetVelocity);
        telemetry.addData("Shooter Actual Velocity: ", robot.shooter.getVelocity());
        telemetry.addData("Shooter Stability Count: ", robot.sequentialStableVelocityChecks);
        telemetry.addData("Offset Angle: ", driverAngle);
        telemetry.addData("Flap Position: ", robot.flapAngle);
        telemetry.addData("Wobble Pot: ", robot.armPot.getVoltage());
        telemetry.addData("Shooter Velocity: ", robot.shooter.getVelocity());
        telemetry.addData("Shooter Stability: ", robot.sequentialStableVelocityChecks);
        telemetry.addData("FL Motor Velocity: ", robot.frontLeft.getVelocity());
        telemetry.addData("FR Motor Velocity: ", robot.frontRight.getVelocity());
        telemetry.addData("RL Motor Velocity: ", robot.rearLeft.getVelocity());
        telemetry.addData("RR Motor Velocity: ", robot.rearRight.getVelocity());
        telemetry.addData("Left Encoder: ", robot.getLeftEncoderWheelPosition());
        telemetry.addData("Strafe Encoder: ", robot.getStrafeEncoderWheelPosition());
        telemetry.addData("Right Encoder: ", robot.getRightEncoderWheelPosition());
        telemetry.addData("Y Power: ", yPower);
        telemetry.addData("X Power: ", xPower);
        telemetry.addData("Spin: ", spin);

        // This prevents us from reading the IMU every loop if we aren't in driver centric.
        if(!robot.disableDriverCentric) {
            telemetry.addData("Gyro Angle: ", robot.readIMU());
        }
        telemetry.addData("Final Auto X Position: ", robot.finalAutoPosition.x);
        telemetry.addData("Final Auto Y Position: ", robot.finalAutoPosition.y);
        telemetry.addData("Final Auto Angle: ", Math.toDegrees(robot.finalAutoPosition.angle));
        telemetry.addData("World X Position: ", MyPosition.worldXPosition);
        telemetry.addData("World Y Position: ", MyPosition.worldYPosition);
        telemetry.addData("World Angle: ", Math.toDegrees(MyPosition.worldAngle_rad));
        telemetry.addData("Loop time: ", loopTime.milliseconds());
        updateTelemetry(telemetry);
    }

    @Override
    public void stop() {
//        robot.stopGroundEffects();
    }
    protected void performActivities() {
        robot.performClawToggle();
        robot.performInjecting();
        robot.performTripleInjecting();
        robot.performShotAligning();
        robot.performRotatingArm();
        robot.updateShooterStability();
    }
}
