package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TESTING TeleOp", group = "TeleOp")
public class TestingTeleOp extends OpMode {
    Robot robot;
    public final double DEADBAND_MAG_NORMAL = 0.1;
    public final double DEADBAND_MAG_SLOW_MODE = 0.03;
    boolean slowModeDrive;
    public boolean willResetIMU = true;

    boolean absHeadingMode = false;

    double loopStartTime = 0;
    double loopEndTime = 0;

    //hungry hippo (1 servo, 2 positions)
    //foundation grabber - latch (2 servos, 2 positions)
    //lift (2 motors, continuous)
    //intake (2 motors, continuous)
    //arm (2 servos, continuous)
    //grabber (1 servo, 2 positions)

    double lastTime;

    Vector2d joystick1, joystick2;

    public void init() {
        robot = new Robot(this, false, false);
        robot.initializeCapstone();
    }

    public void init_loop() {
        if (gamepad1.y) {
            willResetIMU = false;
        }
        lastTime = getRuntime();
    }

    public void start () {
        if (willResetIMU) robot.initIMU();
    }

    public void loop() {
        loopStartTime = System.currentTimeMillis();
        telemetry.addData("OS loop time: ", loopEndTime - loopStartTime);

        robot.updateBulkData(); //read data once per loop, access it through robot class variable
        robot.driveController.updatePositionTracking(telemetry);

        joystick1 = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y); //LEFT joystick
        joystick2 = new Vector2d(gamepad1.right_stick_x, -gamepad1.right_stick_y); //RIGHT joystick
        slowModeDrive = false;

        telemetry.addData("Robot Heading: ", robot.getRobotHeading());
        telemetry.addData("Joystick 2 Angle (180 heading mode): ", joystick2.getAngleDouble(Angle.AngleType.NEG_180_TO_180_HEADING));
        telemetry.addData("Heading to joystick difference: ", joystick2.getAngle().getDifference(robot.getRobotHeading()));

        //slow mode/range stuffs
        if (gamepad1.left_trigger > 0.1) {
            joystick1 = joystick1.scale(0.3);
            joystick2 = joystick2.scale(0.4); //was 0.3
            slowModeDrive = true;
        }
        else if (gamepad1.right_trigger > 0.1) {
            if (robot.getRange(false) < 30) {
                joystick1 = joystick1.scale(0.3);
                joystick2 = joystick2.scale(0.4); //was 0.3
                slowModeDrive = true;
            }
        }

        if (gamepad1.y && gamepad1.b) {
            robot.startTapeMeasure();
        } else {
            robot.stopTapeMeasure();
        }

        robot.driveController.updateUsingJoysticks(
                checkDeadband(joystick1, slowModeDrive).scale(Math.sqrt(2)),
                checkDeadband(joystick2, slowModeDrive).scale(Math.sqrt(2)),
                absHeadingMode
        );


        if (gamepad1.dpad_left) {
            robot.driveController.setDrivingStyle(true);
        } else if (gamepad1.dpad_right) {
            robot.driveController.setDrivingStyle(false);
        }

        if ((gamepad2.x) && (gamepad2.right_trigger > 0.1)) {
            robot.moveLiftToPosition(robot.encoderTicksAtLiftPositions[1] + 10);
            robot.targetPosLift = robot.encoderTicksAtLiftPositions[1] + 10;
            robot.moveGrabberToMid();
        } else {
            robot.moveLift(-gamepad2.left_stick_y);
            if (gamepad2.x) {
                robot.openGrabber();
            } else if (gamepad2.b) {
                robot.closeGrabber();
            }
        }


        double currentTime = getRuntime();
        if (gamepad2.a) {
            robot.currentClawPosition.moveSequence(robot.controller.INSIDE_ROBOT_TO_DELIVERY, currentTime - lastTime);
        }
        if (gamepad2.y) {
            robot.currentClawPosition.moveSequence(robot.controller.DELIVERY_TO_INSIDE_ROBOT, currentTime - lastTime);
        }

        double clawDeltaX = 0;
        double clawDeltaY = 0;

        if ((clawDeltaX != 0) || (clawDeltaY != 0)) {
            robot.currentClawPosition.moveBy(clawDeltaX, clawDeltaY, currentTime - lastTime);
        }

        robot.outtake1.setPosition(robot.currentClawPosition.servoPositions.servo1);
        robot.outtake2.setPosition(robot.currentClawPosition.servoPositions.servo2);

        final double ROTATE_SPEED = 180.0/270;
        robot.outatkeRotatePosition += gamepad2.right_stick_y * ROTATE_SPEED * (currentTime - lastTime);
        if (robot.outatkeRotatePosition > 1) robot.outatkeRotatePosition = 1;
        if (robot.outatkeRotatePosition < 0) robot.outatkeRotatePosition = 0;
        robot.outtakeRot.setPosition(robot.outatkeRotatePosition);

        lastTime = currentTime;

        if (gamepad1.right_bumper) {
            robot.hungryHippoExtend();
        } else if (gamepad1.left_bumper) {
            robot.hungryHippoRetract();
        }

        double intakePower = 0.3;

        if (Math.abs(gamepad2.left_trigger) > 0.1) {
            robot.intake1.setPower(intakePower);
        } else if (gamepad2.left_bumper) {
            robot.intake1.setPower(-intakePower);
        } else {
            robot.intake1.setPower(0);
        }

        if (Math.abs(gamepad2.right_trigger) > 0.1 && !gamepad2.x) {
            robot.intake2.setPower(intakePower);
        } else if (gamepad2.right_bumper) {
            robot.intake2.setPower(-intakePower);
        } else {
            robot.intake2.setPower(0);
        }

        if (gamepad2.dpad_up) {
            robot.hungryHippoExtend();
        } if (gamepad2.dpad_down) {
            robot.hungryHippoRetract();
        }

        //robot.setArmPower(-gamepad2.right_stick_y);
        telemetry.addData("Arm power", -gamepad2.right_stick_x);
        telemetry.addData("lift 1 position: ", robot.bulkData1.getMotorCurrentPosition(robot.lift1));
        telemetry.addData("lift 2 position: ", robot.bulkData1.getMotorCurrentPosition(robot.lift2));

        //SCARA arm back position centered on x: (-76, -185); out of robot: (-76, 150)

        if (gamepad2.right_bumper && gamepad2.right_trigger > 0.6) {
            robot.dropCapstone();
        }

        //todo: remove after done tuning
//        if (gamepad1.b) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE += 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE += 0.01;
//        }
//        if (gamepad1.x) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE -= 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE -= 0.01;
//        }
//
//        if (gamepad2.dpad_right) {
//            robot.LIFT_TICKS_PER_MS += 0.05;
//        } else if (gamepad2.dpad_left) {
//            robot.LIFT_TICKS_PER_MS -= 0.05;
//        }
//
//        if (gamepad1.right_bumper) {
//            robot.driveController.moduleRight.ROBOT_ROTATION_MAX_MAG += 0.01;
//            robot.driveController.moduleLeft.ROBOT_ROTATION_MAX_MAG += 0.01;
//        }
//
//        if (gamepad1.left_bumper) {
//            robot.driveController.moduleRight.ROBOT_ROTATION_MAX_MAG -= 0.01;
//            robot.driveController.moduleLeft.ROBOT_ROTATION_MAX_MAG -= 0.01;
//        }

        telemetry.addData("ROT_ADVANTAGE: ", robot.driveController.moduleLeft.ROT_ADVANTAGE);


        telemetry.addData("joystick 1", joystick1);
        telemetry.addData("joystick 2", joystick2);

        loopEndTime = System.currentTimeMillis();
        telemetry.addData("Our loop time: ", loopEndTime - loopStartTime);

        telemetry.update();
    }

    public void stop() {
        robot.driveController.updateUsingJoysticks(new Vector2d(0, 0), new Vector2d(0, 0), false);
        super.stop();
    }

    public Vector2d checkDeadband(Vector2d joystick, boolean slowMode) {
        return  joystick.getMagnitude() > (slowMode ? DEADBAND_MAG_SLOW_MODE : DEADBAND_MAG_NORMAL) ?
                joystick : new Vector2d(0, 0);
    }
}