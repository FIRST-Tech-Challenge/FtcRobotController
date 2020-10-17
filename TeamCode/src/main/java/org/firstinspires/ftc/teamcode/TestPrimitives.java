package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Point;

import static org.firstinspires.ftc.teamcode.Constants.IntakeState.INTAKE;

@Autonomous(name="Test Primitives", group="Linear Opmode")

public class TestPrimitives extends LinearOpMode {
    Robot robot;
    SimpleTracking simpleTracking;
    SimplePathFollow simplePathFollow;
    SkystoneCV cv;
    boolean isBlue;
    double DEFAULT_POWER = 1; //was 0.6 for most
    double SLOW_POWER = 0.35; //was 0.35

    double TILE = 24 * 2.54;
    double ROBOT = 45;

    double RED_CROSS_FIELD_Y = -95;

    public void runOpMode() {

        robot = new Robot(this, true);
        simpleTracking = new SimpleTracking();
        simplePathFollow = new SimplePathFollow();

        // optionally set starting position and orientation of the robot
        simpleTracking.setOrientationDegrees(0);
        simpleTracking.setPosition(0,0);

        telemetry.addData("WAIT! Initializing IMU.... ", "");
        telemetry.update();

        robot.initIMU();

        cv = new SkystoneCV("Webcam 1", new Point(30, 130), new Point(110, 130), new Point(190, 130), this);
        cv.init(SkystoneCV.CameraType.WEBCAM);

        simpleTracking.setModuleOrientation(robot);

        //robot.setPlacerUp();
        robot.moveGrabberToMid();
        moveSCARA(robot.controller.DELIVERY_TO_INSIDE_ROBOT);

        while (!isStarted()) {
            telemetry.addData("Ready to Run", "");
            telemetry.update();
        }
        double startTime = System.currentTimeMillis();

        while(opModeIsActive()) {
//            moveTo(0, 0, 180, 0.5, 2, 3000);
//            simplePathFollow.stop(robot);

            //robot.setPlacerUp();
            robot.moveGrabberToMid(); //new
            //move lift up
            robot.moveLiftToPosition(200); //was 175

            robot.hungryHippoExtend(); //pull block in
            robot.moveIntake(INTAKE, Constants.IntakeSpeed.SLOW);
            robot.wait(1500, this); //was 1000
            //go forward to grab block
            //moveTo(stonePosition, isBlue ? 25 : -25, isBlue ? 180 : 0, SLOW_POWER, 5, 3000); //was y = +- 40  //was isBlue ? 225 : 315
            simplePathFollow.stop(robot);
            robot.hungryHippoRetract(); //pull hungry hippo back in

            waitForButton();

            robot.moveLiftToPosition(20); //almost all the way down, but don't want to stall
            deliverBlock();

            waitForButton();
        }

    }

    public void deliverBlock () {
        robot.closeGrabber();
        robot.wait(750, this);
        robot.moveLiftToPosition(200); //was 175
        robot.wait(750, this);
        moveSCARA(robot.controller.INSIDE_ROBOT_TO_DELIVERY);
        robot.openGrabber();
        robot.wait(500, this);
        robot.grabberServo.setPosition(0.5);
        //robot.setPlacerUp();
        moveSCARA(robot.controller.DELIVERY_TO_INSIDE_ROBOT);
        robot.moveLiftToPosition(0);
    }

    //move SCARA with sequence
    public void moveSCARA (SCARAController.Sequence sequence) {
        double lastTime = getRuntime();
        double currentTime = getRuntime();
        robot.capstone.setPosition(0);
        while (opModeIsActive() && !robot.currentClawPosition.moveSequence(sequence, currentTime - lastTime)) {
            robot.outtake1.setPosition(robot.currentClawPosition.servoPositions.servo1);
            robot.outtake2.setPosition(robot.currentClawPosition.servoPositions.servo2);

            lastTime = currentTime;
            robot.wait(10, this);
            currentTime = getRuntime();
        }
        // make sure the last position gets sent
        robot.outtake1.setPosition(robot.currentClawPosition.servoPositions.servo1);
        robot.outtake2.setPosition(robot.currentClawPosition.servoPositions.servo2);
    }

    private void moveTo(double x, double y, double orientation, double speed, double threshold, double timeout) {
        boolean done = false;
        double startTime = System.currentTimeMillis();
        while (!done && opModeIsActive() && (System.currentTimeMillis() < startTime + timeout)) {
            robot.updateBulkData();
            simpleTracking.updatePosition(robot);
            done = (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold);
        }
    }

    public void waitForButton () {
        while (!gamepad1.b && opModeIsActive()) {
            simpleTracking.updatePosition(robot);
            telemetry.addData("IMU Heading: ", robot.getRobotHeading());
        }
    }

}
