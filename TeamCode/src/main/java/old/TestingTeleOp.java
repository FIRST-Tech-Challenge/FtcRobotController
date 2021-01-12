package old;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestingTeleOp", group = "TeleOp")
public class TestingTeleOp extends OpMode {
    Robot robot;
    public final double DEADBAND_MAG_NORMAL = 0.1;
    public final double DEADBAND_MAG_SLOW_MODE = 0.03;
    boolean slowModeDrive;
    public boolean willResetIMU = true;

    boolean absHeadingMode = false;

    double loopStartTime = 0;
    double loopEndTime = 0;

    double startTime = 0;
    double mathTime = 0;

    //hungry hippo (1 servo, 2 positions)
    //foundation grabber - latch (2 servos, 2 positions)
    //lift (2 motors, continuous)
    //intake (2 motors, continuous)
    //arm (2 servos, continuous)
    //grabber (1 servo, 2 positions)

    double lastTime;

    Vector2d joystick1, joystick2;

    public void init() {
        robot = new Robot(this, true, false);
    }

    public void init_loop() {
        if (gamepad1.y) {
            willResetIMU = false;
        }
        lastTime = getRuntime();
    }


    public void start () {
        if (willResetIMU) robot.initIMU();
        startTime = System.currentTimeMillis();
    }

    public void loop() {
        loopStartTime = System.currentTimeMillis();
        mathTime = (loopStartTime-startTime)/1000;
        telemetry.addData("OS loop time: ", loopEndTime - loopStartTime);

        robot.updateBulkData(); //read data once per loop, access it through robot class variable
        robot.driveController.updatePositionTracking(telemetry);

        joystick1 = new Vector2d(Math.cos(mathTime), Math.sin(mathTime)); //LEFT joystick
        joystick2 = new Vector2d(gamepad1.right_stick_x, -gamepad1.right_stick_y); //RIGHT joystick
        slowModeDrive = false;

        telemetry.addData("Robot Heading: ", robot.getRobotHeading());

        telemetry.addData("Joystick 2 Angle (180 heading mode): ", joystick2.getAngleDouble(Angle.AngleType.NEG_180_TO_180_HEADING));
        telemetry.addData("Heading to joystick difference: ", joystick2.getAngle().getDifference(robot.getRobotHeading()));

        telemetry.addData("Left Orientation: ", robot.driveController.moduleLeft.getCurrentOrientation());
        telemetry.addData("Right Orientation: ", robot.driveController.moduleRight.getCurrentOrientation());

        telemetry.addData("StartTime: ", startTime);
        telemetry.addData("MathTime: ", mathTime);



        //slow mode/range stuffs
        if (gamepad1.left_trigger > 0.1) {
            // joystick1 = joystick1.scale(0.3);
            // joystick2 = joystick2.scale(0.4); //was 0.3
            joystick1 = joystick1.scale((1-Math.abs(gamepad1.left_trigger))*.75);
            joystick2 = joystick2.scale(1-Math.abs(gamepad1.left_trigger));
            slowModeDrive = true;
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