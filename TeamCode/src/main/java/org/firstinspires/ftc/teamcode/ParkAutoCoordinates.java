package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ParkAutoCoordinates extends LinearOpMode {
    boolean parkRight;
    Robot robot;
    SimpleTracking simpleTracking;
    SimplePathFollow simplePathFollow;
    double PARK_DISTANCE = 20; //cm; was 40

    public ParkAutoCoordinates(boolean parkRight){
        this.parkRight = parkRight;
    }
    public void runOpMode(){
        robot = new Robot(this, true);
        simpleTracking = new SimpleTracking();
        simplePathFollow = new SimplePathFollow();

        //set starting position to (0, 0) and heading to 0 to simplify
        simpleTracking.setPosition(0, 0);
        simpleTracking.setOrientationDegrees(0);
        simpleTracking.setModuleOrientation(robot);

        telemetry.addData("WAIT! Initializing IMU.... ", "");
        telemetry.update();
        robot.initIMU();

        while (!isStarted()) {
            telemetry.addData("Ready to Run", "");
            telemetry.update();
        }

        moveTo(parkRight ? PARK_DISTANCE : -PARK_DISTANCE, 0, 0);

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
    private void moveTo(double x, double y, double orientation) {
        moveTo(x, y, orientation, 0.7, 5);
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
