package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class FoundationAuto extends LinearOpMode {

    boolean willWait;
    boolean isBlue = true;
    Robot robot;
    SimpleTracking simpleTracking;
    SimplePathFollow simplePathFollow;
    FieldTracker fieldTracker;
    SkystoneCV cv;
    double TILE = 24 * 2.54;
    double ROBOT = 45;
    double currentPosX = 0;
    double currentPosY = 0;
    double stonePosition;
    double DEFAULT_POWER = 0.8; //was 0.6 for most

    FoundationAuto(boolean isBlue, boolean willWait) {
        this.isBlue = isBlue;
        this.willWait = willWait;
    }

    FoundationAuto(boolean isBlue) { this(isBlue, false); }

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

        simpleTracking.setModuleOrientation(robot);

        while (!isStarted()) {
            telemetry.addData("Ready to Run", "");
            telemetry.update();
        }

        // blue starting position
        simpleTracking.setPosition(90, isBlue ? 157.5 : -157.5);
        simpleTracking.setOrientationDegrees(isBlue ? 180 : 0);

        if (willWait) { robot.wait(15000, this); }

        moveTo(120, 100 * (isBlue ? 1 : -1), isBlue ? 180 : 0, DEFAULT_POWER, 5);
        moveTo(120, 100 * (isBlue ? 1 : -1), isBlue ? 0 : 180, DEFAULT_POWER, 5);

        moveWithRangeSensorTo(120, 35 * (isBlue ? 1 : -1), isBlue ? 0 : 180, 0.3, 5, 3000);

        robot.latch();

        robot.wait(1000, this);

        moveTo(120, isBlue ? 115 : -115, isBlue ? 0 : 180, 0.6, 5, 5000); //was y = 125
        simplePathFollow.stop(robot);
        moveTo(60, 95 * (isBlue ? 1 : -1), isBlue ? 245 : 295, DEFAULT_POWER, 2, 2500); //pivot platform
        moveTo(90, isBlue ? 100 : -100, isBlue ? 245 : 295, DEFAULT_POWER, 5, 2000); //push platform

        robot.unlatch();
        robot.wait(1000, this);

        moveTo(-20, 140 * (isBlue ? 1 : -1), isBlue ? 0 : 180, DEFAULT_POWER, 5, 2000);



        /*

        moveTo(90, isBlue ? 130 : -130, isBlue ? 180 : 0, .6, 5);
        moveTo(90, isBlue ? 130 : -130, isBlue ? 0 : 180, .6, 5);

        moveTo(TILE * 3, isBlue ? 130 : -130, isBlue ? 0 : 180, .6, 5);
        //todo copy sequence from reg auto
        moveTo(120, isBlue ? 90: -90, isBlue ? 0:180, DEFAULT_POWER, 5);
        simplePathFollow.stop(robot);
        moveWithRangeSensorTo(120, isBlue ? 35: -35, isBlue ? 0 : 180, 0.4, 5, 3000);
//        moveTo(isBlue ? 120 : -213, 60, 0, 0.4, 5);
        simplePathFollow.stop(robot);
        robot.latchServo1.setPosition(0.0);
        robot.latchServo2.setPosition(1.0);
//        robot.wait(200, this, simpleTracking);   //REMOVED 1-20
//        robot.closeGrabber();
//        robot.wait(200, this, simpleTracking);
//        robot.openGrabber();
//        robot.wait(200, this, simpleTracking);
//        robot.closeGrabber();
//        robot.wait(500, this, simpleTracking);
//        robot.armServo1.setPosition(.3);
//        robot.armServo2.setPosition(.7);
//        robot.wait(1200, this, simpleTracking);
//        robot.openGrabber();
//        robot.armServo1.setPosition(.7);
//        robot.armServo2.setPosition(.3);

//        robot.wait(1000, this, simpleTracking);  //REMOVED 1-20
//        robot.armServo1.setPosition(.5);
//        robot.armServo2.setPosition(.5);
        moveTo(120, isBlue ? 125:-125, isBlue ?0:180, 0.6, 5, 5000);
        simplePathFollow.stop(robot);
        moveTo(50, isBlue ? 125 : -125, isBlue ? 245 : 295, 1.0, 2); //pivot platform  //WAS 80
        moveTo(110, isBlue ? 125 : -125, 270, DEFAULT_POWER, 5, 2000);
        simplePathFollow.stop(robot);
        robot.latchServo1.setPosition(1.0);
        robot.latchServo2.setPosition(0.0);
        //todo park (values might not be good here)
        moveTo(90, isBlue ? 95 : -95, 270, DEFAULT_POWER, 5);//todo the 95 is from measuring tape, may run into bridge
        moveTo(0, isBlue ? 95 : -95, 270, DEFAULT_POWER, 5);

*/

    }
    private void moveTo(double x, double y, double orientation, double speed, double threshold) {
        boolean done = false;
        while (!done && opModeIsActive()) {
            robot.updateBulkData();
            simpleTracking.updatePosition(robot);
            telemetry.addData("Using", "encoders");
            //telemetry.update();
            done = (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold);
            logTelemetry();
        }
    }

    private void moveWithIMU(double x, double y, double orientation, double speed, double threshold) {
        boolean done = false;
        while (!done && opModeIsActive()) {
            robot.updateBulkData();
//            simpleTracking.updatePosition(robot, true, isBlue);
            simpleTracking.updatePosition(robot);
            telemetry.addData("Using", "encoders");
            //telemetry.update();
            done = (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold);
            logTelemetry();
        }
    }

    // with timeout
    private void moveTo(double x, double y, double orientation, double speed, double threshold, double timeout) {
        boolean done = false;
        double startTime = System.currentTimeMillis();
        while (!done && opModeIsActive() && (System.currentTimeMillis() < startTime + timeout)) {
            robot.updateBulkData();
            simpleTracking.updatePosition(robot);
            done = (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold);
            logTelemetry();
        }
    }

    private void moveWithRangeSensorTo(double x, double y, double orientation, double speed, double threshold, double timeout) {
        boolean done = false;
        double startTime = System.currentTimeMillis();
        while (!done && opModeIsActive() && (System.currentTimeMillis() < startTime + timeout)) {
            robot.updateBulkData();
            simpleTracking.updatePosition(robot);
            done = (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold) ||
                    ( robot.backRangeSensor.getDistance(DistanceUnit.CM) < 5);
            logTelemetry();
        }
    }

    private void moveAndUpdateSCARA(double x, double y, double orientation, double speed, double threshold, double scaraDistance) {
        boolean done = false;
        boolean scaraDone = false;
        double currentTime;
        double lastTime = getRuntime();
        while ((!done || !scaraDone) && opModeIsActive()) {
            robot.updateBulkData();
            if (!done) {
                simpleTracking.updatePosition(robot);
                telemetry.addData("Using", "encoders");
                //telemetry.update();
                done = (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold);
            }
            if (!scaraDone) {
                currentTime = getRuntime();
                scaraDone = robot.currentClawPosition.moveTo(SCARAController.MIDLINE, scaraDistance, currentTime - lastTime);
                lastTime = currentTime;
            }
            logTelemetry();
        }
    }

    //move relative to the current position
    private void move(double x, double y, double orientation, double speed, double threshold) {
        moveTo(simpleTracking.lastRobotX + x, simpleTracking.lastRobotY + y, simpleTracking.lastRobotOrientation + orientation, speed, threshold);
    }

    public void updateOrientationWithIMU(boolean isBlue) {
        double angle =  robot.getRobotHeading().getAngle();
        if (isBlue) angle += 180;
        if (angle < 0) angle += 360;
        if (angle >= 360) angle -= 360;
        simpleTracking.setOrientationDegrees(angle);
    }

    public void waitForButton () {
        while (!gamepad1.b && opModeIsActive()) {
            simpleTracking.updatePosition(robot);
            telemetry.addData("Vuforia location: \n", fieldTracker.getTargetInfo());
            telemetry.addData("IMU Heading: ", robot.getRobotHeading());
            logTelemetry();
        }
    }

    private void logTelemetry() {
        telemetry.addData("Left Top encoder", simpleTracking.leftTopMotorPosition);
        telemetry.addData("Left Bottom encoder", simpleTracking.leftBottomMotorPosition);
        telemetry.addData("Left Distance", simpleTracking.lastleftDistance);
        telemetry.addData("Left Orientation", simpleTracking.lastLeftOrientation);

        telemetry.addData("Right Top encoder", simpleTracking.rightTopMotorPosition);
        telemetry.addData("Right Bottom encoder", simpleTracking.rightBottomMotorPosition);
        telemetry.addData("Right Distance", simpleTracking.lastRightDistance);
        telemetry.addData("Right Orientation", simpleTracking.lastRightOrientation);

        telemetry.addData("Target translation", simplePathFollow.targetTranslation.x + ", " + simplePathFollow.targetTranslation.y);
        telemetry.addData("Target rotation", simplePathFollow.targetRotation.x + ", " + simplePathFollow.targetRotation.y);
        telemetry.addData("Target Left Power", simplePathFollow.targetLeftPower.x + ", " + simplePathFollow.targetLeftPower.y);
        telemetry.addData("Target Right Power", simplePathFollow.targetRightPower.x + ", " + simplePathFollow.targetRightPower.y);

        telemetry.addData("Left Top Power", simplePathFollow.leftTopPower);
        telemetry.addData("Left Bottom Power", simplePathFollow.leftBottomPower);
        telemetry.addData("Right Top Power", simplePathFollow.rightTopPower);
        telemetry.addData("Right Bottom Power", simplePathFollow.rightBottomPower);

        telemetry.addData("Robot position", simpleTracking.lastRobotX + ", " + simpleTracking.lastRobotY);
        telemetry.addData("Robot orientation", simpleTracking.lastRobotOrientation);

        telemetry.update();
    }
}
