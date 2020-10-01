package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;

import static java.lang.Math.*;

/**
 * Created by 12090 STEM Punk.
 */

@TeleOp(name="Omni: TeleOp", group ="TeleOp")
public class OmniTeleOp extends OpMode {

    public HardwareOmnibot robot = new HardwareOmnibot();
    public OmniTeleOp() {
        msStuckDetectInit = 10000;
    }

    public enum CompleteActivities {
        LIFT,
        RELEASE,
        STOW
    }

    @Override
    public void init() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        robot.init(hardwareMap);
//        robot.disableDriveEncoders();
        robot.setInputShaping(true);
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Press X on driver controller to reset encoders.");
        if(gamepad1.x) {
            robot.forceReset = true;
        }
    }

    private CompleteActivities completeActivities = CompleteActivities.STOW;
    private double driverAngle = 0.0;
    private final double MAX_SPEED = 1.0;
    private final double MAX_SPIN = 1.0;
    private final double FOUNDATION_SPEED = 0.40;
    private final double FOUNDATION_SPIN = 0.40;
    private double speedMultiplier = MAX_SPEED;
    private double spinMultiplier = MAX_SPIN;
    private int heightIncrement = 10;
    private boolean aHeld = false;
    private boolean bHeld = false;
    private boolean yHeld = false;
    private boolean upHeld = false;
    private boolean downHeld = false;
    private boolean leftHeld = false;
    private boolean rightHeld = false;
    private boolean leftBumperHeld = false;
    private boolean rightBumperHeld = false;
    private boolean a2Held = false;
    private boolean b2Held = false;
    private boolean y2Held = false;
    private boolean x2Held = false;
    private boolean up2Held = false;
    private boolean down2Held = false;
    private boolean left2Held = false;
    private boolean right2Held = false;
    private boolean leftBumper2Held = false;
    private boolean rightBumper2Held = false;
    private boolean aPressed;
    private boolean bPressed;
    private boolean yPressed;
    private boolean leftPressed;
    private boolean rightPressed;
    private boolean leftBumperPressed;
    private boolean rightBumperPressed;
    private boolean a2Pressed;
    private boolean b2Pressed;
    private boolean y2Pressed;
    private boolean x2Pressed;
    private boolean up2Pressed;
    private boolean down2Pressed;
    private boolean left2Pressed;
    private boolean right2Pressed;
    private boolean leftBumper2Pressed;
    private boolean rightBumper2Pressed;
    private boolean fingersUp = true;
    private double yPower;
    private double xPower;
    private double spin;
    private double gyroAngle;
    private int liftEncoderSetpoint = 0;
    private ElapsedTime loopTime = new ElapsedTime();
    private boolean runWithEncoders = true;
    private boolean aligning = false;
    private boolean scissorExtended = false;


    @Override
    public void start()
    {
        robot.resetEncoders();

        //give MyPosition our current positions so that it saves the last positions of the wheels
        //this means we won't teleport when we start the match. Just in case, run this twice
        for(int i = 0; i < 2 ; i ++){
            robot.resetReads();
            MyPosition.initialize(robot.getLeftEncoderWheelPosition(),
                    robot.getRightEncoderWheelPosition(),
                    robot.getStrafeEncoderWheelPosition());
        }
//        MyPosition.setPosition(335.915, 83.14436, Math.toRadians(180.0));
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
        gyroAngle = robot.readIMU();

        yPower = -HardwareOmnibot.cleanMotionValues(gamepad1.left_stick_y);
        xPower = HardwareOmnibot.cleanMotionValues(gamepad1.left_stick_x);
        // GF used the angle system where rotating left was positive, so have to reverse the sign
        // of the joystick.
        spin = -HardwareOmnibot.cleanMotionValues(gamepad1.right_stick_x);
        aPressed = gamepad1.a;
        bPressed = gamepad1.b;
        yPressed = gamepad1.y;
		rightPressed = gamepad1.dpad_right;
		leftPressed = gamepad1.dpad_left;
		leftBumperPressed = gamepad1.left_bumper;
        rightBumperPressed = gamepad1.right_bumper;
        a2Pressed = gamepad2.a;
        b2Pressed = gamepad2.b;
        y2Pressed = gamepad2.y;
        x2Pressed = gamepad2.x;
        up2Pressed = gamepad2.dpad_up;
        down2Pressed = gamepad2.dpad_down;
        right2Pressed = gamepad2.dpad_right;
        left2Pressed = gamepad2.dpad_left;
        leftBumper2Pressed = gamepad2.left_bumper;
        rightBumper2Pressed = gamepad2.right_bumper;

        if (gamepad1.x) {
            // The driver presses X, then uses the left joystick to say what angle the robot
            // is aiming.  This will calculate the values as long as X is pressed, and will
            // not drive the robot using the left stick.  Once X is released, it will use the
            // final calculated angle and drive with the left stick.  Button should be released
            // before stick.  The default behavior of atan2 is 0 to -180 on Y Axis CCW, and 0 to
            // 180 CW.  This code normalizes that to 0 to 360 CCW from the Y Axis
            //robot.resetGyro();
            // This was -90, but needed to be oriented 90 degrees from actual angle.
            driverAngle = toDegrees(atan2(yPower, xPower))- 0.0 - robot.readIMU();
            xPower = 0.0;
            yPower = 0.0;
            spin = 0.0;
        }

		// ********************************************************************
		// DRIVER JOYSTICK
		// ********************************************************************
        if(!bHeld && bPressed)
        {
            if(speedMultiplier == MAX_SPEED) {
                speedMultiplier = FOUNDATION_SPEED;
                spinMultiplier = FOUNDATION_SPIN;
                robot.strafeMultiplier = HardwareOmnibot.SLOW_STRAFE_MULTIPLIER;
            } else {
                speedMultiplier = MAX_SPEED;
                spinMultiplier = MAX_SPIN;
                robot.strafeMultiplier = HardwareOmnibot.STRAFE_MULTIPLIER;
            }
            bHeld = true;
        } else if(!bPressed) {
            bHeld = false;
        }

        if(!yHeld && yPressed)
        {
            if(fingersUp) {
                robot.fingersDown();
                fingersUp = false;
            } else {
                robot.fingersUp();
                fingersUp = true;
            }
            yHeld = true;
        } else if(!yPressed) {
            yHeld = false;
        }

        if(!aHeld && aPressed)
        {
            aHeld = true;
			if(robot.intakePower != 0.0) {
				robot.stopIntake();
			} else {
				robot.startIntake(false);
			}
        } else if(!aPressed) {
            aHeld = false;
        }

        if(!rightHeld && rightPressed)
        {
            if(!aligning) {
                aligning = robot.startStackAligning();
            } else {
                robot.stopStackAligning();
                aligning = false;
            }
            rightHeld = true;
        } else if(!rightPressed) {
            rightHeld = false;
        }

        if(!leftHeld && leftPressed)
        {
            leftHeld = true;
        } else if(!leftPressed) {
            leftHeld = false;
        }

        if(!rightBumperHeld && rightBumperPressed)
        {
            robot.startExtendingIntake();
            rightBumperHeld = true;
        } else if(!rightBumperPressed) {
            rightBumperHeld = false;
        }

        if(!leftBumperHeld && leftBumperPressed)
        {
            robot.extender.setPower(-1.0);
            leftBumperHeld = true;
        } else if (leftBumperHeld && !leftBumperPressed) {
            robot.extender.setPower(0.0);
            leftBumperHeld = false;
        }
        else if(!leftBumperPressed) {
            leftBumperHeld = false;
        }

		// ********************************************************************
		// OPERATOR JOYSTICK
		// ********************************************************************
		// This was unassigned (fingers up/down)
        if(!x2Held && x2Pressed)
        {
            robot.startEjecting();
            x2Held = true;
        } else if(!x2Pressed) {
            x2Held = false;
        }

        if(!a2Held && a2Pressed)
        {
            a2Held = true;
            switch(completeActivities) {
                case LIFT:
                    if(robot.startLifting()) {
                        completeActivities = CompleteActivities.RELEASE;
                    }
                    break;
                case RELEASE:
                    if (robot.startReleasing() ) {
                        completeActivities = CompleteActivities.STOW;
                    }
                    break;
                case STOW:
                    if (robot.startStowing() ) {
                        completeActivities = CompleteActivities.LIFT;
                    }
                    break;
            }
        } else if(!a2Pressed) {
            a2Held = false;
        }

        if(!b2Held && b2Pressed)
        {
            b2Held = true;
            robot.startCapstone();
        } else if(!b2Pressed) {
            b2Held = false;
        }

        if(!y2Held && y2Pressed)
        {
            y2Held = true;
            if(!scissorExtended) {
                robot.startExtendScissor();
                scissorExtended = true;
            } else {
                robot.scissorExtender.setPosition(HardwareOmnibot.SCISSOR_CLOSED);
                scissorExtended = false;
            }
        } else if(!y2Pressed) {
            y2Held = false;
        }

        if(!left2Held && left2Pressed)
        {
            left2Held = true;
        } else if (!left2Pressed) {
            left2Held = false;
        }

        if(!right2Held && right2Pressed)
        {
            right2Held = true;
        } else if (!right2Pressed) {
            right2Held = false;
        }

        if(!up2Held && up2Pressed)
        {
            up2Held = true;
            robot.addStone();
            robot.adjustLiftHeight();
        } else if (!up2Pressed) {
			up2Held = false;
		}

        if(!down2Held && down2Pressed)
        {
            down2Held = true;
            robot.removeStone();
            robot.adjustLiftHeight();
        } else if (!down2Pressed) {
			down2Held = false;
		}

        if(!rightBumper2Held && rightBumper2Pressed)
        {
            liftEncoderSetpoint = robot.getLifterPosition();
            liftEncoderSetpoint += heightIncrement;
            robot.lifter.setTargetPosition(liftEncoderSetpoint);
            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lifter.setPower(robot.LIFT_MAX_SPEED);
            rightBumper2Held = true;
        } else if(!rightBumper2Pressed) {
            rightBumper2Held = false;
        }

        if(!leftBumper2Held && leftBumper2Pressed)
        {
            liftEncoderSetpoint = robot.getLifterPosition();
            liftEncoderSetpoint -= heightIncrement;
            robot.lifter.setTargetPosition(liftEncoderSetpoint);
            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lifter.setPower(robot.LIFT_MIN_SPEED);
            leftBumper2Held = true;
        } else if(!leftBumper2Pressed) {
            leftBumper2Held = false;
        }

        // If the activity is not performing, it will be idle and return.
        robot.performLifting();
        robot.performReleasing();
        robot.performStowing();
        robot.performEjecting();
        robot.performExtendingIntake();
        robot.performCapstone();
        robot.performStackAligning();
        robot.performExtendScissor();

        if(robot.stackAlignmentState == HardwareOmnibot.StackAlignActivity.IDLE) {
            aligning = false;
            robot.drive(speedMultiplier * xPower, speedMultiplier * yPower,
                    spinMultiplier * spin, driverAngle, robot.defaultInputShaping);
        }

		telemetry.addData("Lift Target Height: ", robot.liftTargetHeight);
        telemetry.addData("Offset Angle: ", driverAngle);

        telemetry.addData("Align State: ", robot.stackAlignmentState);
        telemetry.addData("Lift State: ", robot.liftState);
        telemetry.addData("Release State: ", robot.releaseState);
        telemetry.addData("Stow State: ", robot.stowState);
        telemetry.addData("Eject State: ", robot.ejectState);
        telemetry.addData("Capstone State: ", robot.capstoneState);
        telemetry.addData("Stack State: ", robot.stackStone);
        telemetry.addData("Extend State: ", robot.extendState);

        telemetry.addData("Stone Detector: ", robot.stoneDetector.getVoltage());
        telemetry.addData("Stone Present: ", robot.stonePresent());
        telemetry.addData("Lift Position: ", robot.getLifterPosition());
        telemetry.addData("Left Encoder: ", robot.getLeftEncoderWheelPosition());
        telemetry.addData("Strafe Encoder: ", robot.getStrafeEncoderWheelPosition());
        telemetry.addData("Right Encoder: ", robot.getRightEncoderWheelPosition());
        telemetry.addData("Extender Limit Switch: ", robot.intakeLimit.getState());
        telemetry.addData("Y Power: ", yPower);
        telemetry.addData("X Power: ", xPower);
        telemetry.addData("Spin: ", spin);
        telemetry.addData("Gyro Angle: ", gyroAngle);
        telemetry.addData("World X Position: ", MyPosition.worldXPosition);
        telemetry.addData("World Y Position: ", MyPosition.worldYPosition);
        telemetry.addData("World Angle: ", Math.toDegrees(MyPosition.worldAngle_rad));
        telemetry.addData("Loop time: ", loopTime.milliseconds());
        updateTelemetry(telemetry);
    }

    @Override
    public void stop() {
        robot.stopGroundEffects();
    }
}
