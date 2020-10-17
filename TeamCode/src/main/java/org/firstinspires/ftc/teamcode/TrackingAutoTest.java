package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Point;

import static org.firstinspires.ftc.teamcode.Constants.IntakeState.INTAKE;
import static org.firstinspires.ftc.teamcode.Constants.IntakeState.OUTTAKE;
import static org.firstinspires.ftc.teamcode.Constants.IntakeState.STOP;

@Autonomous(name = "Tracking Test", group = "Linear Opmode")
public class TrackingAutoTest extends LinearOpMode {

    public TrackingAutoTest(boolean isBlue, boolean twoSkystone, boolean deliverWithScara) {
        this.isBlue = isBlue;
        this.twoSkystone = twoSkystone;
        this.deliverWithScara = deliverWithScara;
    }

    Robot robot;
    SimpleTracking simpleTracking;
    SimplePathFollow simplePathFollow;
    SkystoneCV cv;
    boolean isBlue, twoSkystone, deliverWithScara;
    double DEFAULT_POWER = 1.5; //was 0.6 for most
    double MID_POWER = 1.5; //was 0.75
    double SLOW_POWER = 0.4; //was 0.35


    public void runOpMode() {
        robot = new Robot(this, true);
        simpleTracking = new SimpleTracking();
        simplePathFollow = new SimplePathFollow();

        // set starting position and orientation of the robot
        simpleTracking.setPosition(-90, isBlue ? 157.5 : -157.5);
        simpleTracking.setOrientationDegrees(isBlue ? 180 : 0);

        if (!twoSkystone) {
            MID_POWER = 0.7;
            DEFAULT_POWER = 0.8;
        }

        robot.initializeCapstone();

        telemetry.addData("WAIT! Initializing IMU.... ", "");
        telemetry.update();

        robot.initIMU();

        cv = new SkystoneCV("Webcam 1", new Point(30, 130), new Point(110, 130), new Point(190, 130), this);
        cv.init(SkystoneCV.CameraType.WEBCAM);

        simpleTracking.setModuleOrientation(robot);

        if (deliverWithScara) {
            prepareToGrab();
        }

        while (!isStarted()) {
            telemetry.addData("Ready to Run", "");
            telemetry.update();
        }
        long startTime = System.currentTimeMillis();

        //START AUTO

        // set SCARA to be inside robot
        moveSCARA(robot.controller.DELIVERY_TO_INSIDE_ROBOT);

        //identify skystone
        SkystoneCV.StonePosition skystonePosition = cv.getSkystonePosition();
        telemetry.addData("Skystone at: ", skystonePosition);
        telemetry.update();

        double stoneOffset;
        double stonePosition;

        if (isBlue) {
            stoneOffset = 0;
            stonePosition = -70 + stoneOffset; //was +5
            if (skystonePosition == SkystoneCV.StonePosition.CENTER) {
                stonePosition -= 8 * 2.54 - 5;
            }
            if (skystonePosition == SkystoneCV.StonePosition.RIGHT) {
                stonePosition -= 16 * 2.54;
            }
        } else {
            stoneOffset = 0;
            stonePosition = -70 + stoneOffset; //was +5
            if (skystonePosition == SkystoneCV.StonePosition.CENTER) {
                stonePosition -= 8 * 2.54 - 5 + 2.5;
            }
            if (skystonePosition == SkystoneCV.StonePosition.LEFT) {
                stonePosition -= 16 * 2.54;
            }
        }

        //close openCV
        cv.camera.stopStreaming();
        cv.camera.closeCameraDevice();

        if (deliverWithScara) {
            //move lift up
            robot.moveLiftToPosition(175);
        }

        // go to first stone
        //moveTo(stonePosition, (isBlue ? 75 : -75), isBlue ? 225 : 315, 0.5, 5, 3000);
        moveTo(stonePosition, (isBlue ? 85 : -85), isBlue ? 180 : 0, 0.5, 5, 3000); //was 75 //was 85 //was 90
        simplePathFollow.stop(robot);

        robot.hungryHippoExtend();

        robot.moveIntake(INTAKE, Constants.IntakeSpeed.SLOW);
        //go forward to grab block
        //moveTo(stonePosition, isBlue ? 25 : -25, isBlue ? 225 : 315, SLOW_POWER, 5, 3000); //was y = +- 40
        simplePathFollow.stop(robot);

        //intake stone
        robot.wait(1000, this); //was 1000
        robot.hungryHippoRetract();

        //removed 3-7... added back... re-removed 3/9
        //moveTo(stonePosition, (isBlue ? 75 : -75), isBlue ? 180 : 0, 0.5, 5, 3000); //was 75 //was 85

        double firstYDistance = 100; //was 95 //was 100

        moveTo(stonePosition, firstYDistance * (isBlue ? 1 : -1), isBlue ? 180 : 0, 0.5, 2); //WAS 95 //was 90

        robot.wait(500, this); //added 3-2

        if (deliverWithScara) {
            robot.moveLiftToPosition(20); //lift down
        }

        moveTo(stonePosition, firstYDistance * (isBlue ? 1 : -1), 270, 0.7, 1); //WAS ALSO 95 //was 90 //was 0.5 speed
        simplePathFollow.stop(robot);

        // move to platform
        double crossFieldX = 75;
        moveWithIMU(crossFieldX, (firstYDistance /*+ 15*/) * (isBlue ? 1 : -1), 270, MID_POWER, 5, 6000); //was x = 120 //was 0.6 power //was 90 //was y = 110
        moveTo(120, firstYDistance * (isBlue ? 1 : -1), isBlue ? 0 : 180, DEFAULT_POWER, 5); //was 90
        simplePathFollow.stop(robot);
        if (twoSkystone) moveWithRangeSensorTo(120, (35 + 5) * (isBlue ? 1 : -1), isBlue ? 0 : 180, MID_POWER, 5, 3000);
        else moveWithRangeSensorTo(120, 35 * (isBlue ? 1 : -1), isBlue ? 0 : 180, SLOW_POWER, 5, 3000);
        simplePathFollow.stop(robot);

        if (!twoSkystone) latchHooksDown();

        if (deliverWithScara) {
            //deliver block to foundation
            deliverBlock(250);
        }

        robot.moveIntake(STOP);

        //outtake just in case
        robot.moveIntake(OUTTAKE);
        //robot.wait(750, this); //was 1500 timeout

        if (!twoSkystone) { //was !deliverWithScara (?)
            //move foundation
            moveTo(120, isBlue ? 115 : -115, isBlue ? 0 : 180, DEFAULT_POWER, 5, 5000); //was y = 125
            simplePathFollow.stop(robot);
            moveTo(60, 95 * (isBlue ? 1 : -1), isBlue ? 245 : 295, DEFAULT_POWER, 2, 2500); //pivot platform  //WAS x = 50 //WAS angle = isBlue ? 225 : 315
            //moveTo(110, isBlue ? 140 : -140, 270, DEFAULT_POWER, 5, 2000); //push platform
            moveTo(90, isBlue ? 100 : -100, isBlue ? 245 : 295, DEFAULT_POWER, 5, 2000); //push platform
            robot.moveIntake(STOP);
            simplePathFollow.stop(robot);
            latchHooksUp();
        }


        //TWO SKYSTONE
        if (deliverWithScara) prepareToGrab();
        if (twoSkystone /*&& System.currentTimeMillis() - startTime < 25*/) { //at least 5 seconds left
            double skystoneDistX = -100 - 10; //was -100 // was -100 + 10 //was -100 - 5
            if (skystonePosition == SkystoneCV.StonePosition.CENTER) {
                skystoneDistX -= 8*2.54;
            } if ((skystonePosition == SkystoneCV.StonePosition.RIGHT && isBlue) || (skystonePosition == SkystoneCV.StonePosition.LEFT && !isBlue)) {
                skystoneDistX -= 16*2.54;
            }

            double secondYDistance = firstYDistance; //was 80 //was 85
            double getBlockYDistance = 40; //30 -> 35 -> 40
            moveTo(50, secondYDistance * (isBlue ? 1 : -1), 270, MID_POWER, 5); //position for cross-field drive (was x = 100) //was y = 90

            moveWithIMU(skystoneDistX, (secondYDistance /*+ 10*/) * (isBlue ? 1 : -1), 270, MID_POWER, 5); //align next to stone (was x = 95)

            if (deliverWithScara) robot.moveLiftToPosition(175);
            else robot.moveLiftToPosition(0);

            //move into block line
            moveTo(skystoneDistX, getBlockYDistance * (isBlue ? 1 : -1), 270, MID_POWER, 5); //was y = 57 //was y = 50 //was 0.6 power
            robot.moveIntake(INTAKE, Constants.IntakeSpeed.SLOW);

            //drive forward to intake block
            moveTo(skystoneDistX - 10, getBlockYDistance * (isBlue ? 1 : -1), 270, DEFAULT_POWER, 5); //was 0.3 power //was skyDist - 10
            robot.wait(750, this); //wait for block to intake
            moveTo(skystoneDistX - 10, secondYDistance * (isBlue ? 1 : -1), 270, MID_POWER, 5); //was x-20 //was x
            //robot.moveIntake(STOP); //moved one line down (give more time to intake)

            if (deliverWithScara) robot.moveLiftToPosition(20);

            double secondStoneXOffset = 20; //30 -> 35 -> 20
            //move back to original (unmoved) foundation position
            moveWithIMU(crossFieldX - secondStoneXOffset, (firstYDistance /*+ 15*/) * (isBlue ? 1 : -1), 270, MID_POWER, 5, 6000); //was x = 120 //was 0.6 power //was 90 //was y = 110 //was x=95, x=80
            moveTo(120 - secondStoneXOffset, firstYDistance * (isBlue ? 1 : -1), isBlue ? 0 : 180, DEFAULT_POWER, 5); //was 90
            simplePathFollow.stop(robot);
            moveWithRangeSensorTo(120 - secondStoneXOffset, (35 + 5) * (isBlue ? 1 : -1), isBlue ? 0 : 180, 0.8, 5, 3000);
            simplePathFollow.stop(robot);
            robot.moveIntake(STOP);

            latchHooksDown();
            if (deliverWithScara) {
                //deliver block to foundation
                //deliverBlock(350); //first delivery is 250
            } else {
                robot.wait(500, this);
            }

            //outtake just in case (or for no scara delivery)
            robot.moveIntake(OUTTAKE);
            //robot.wait(1000, this); //was 1500 timeout

            //move foundation sequence
            //robot.wait(500, this);

            robot.closeGrabber();
            moveTo(120 - secondStoneXOffset, isBlue ? 115 : -115, isBlue ? 0 : 180, DEFAULT_POWER, 5, 5000); //was y = 125
            //simplePathFollow.stop(robot);

            robot.moveLiftToPosition(350); //was 175
            moveTo(60 - secondStoneXOffset, 95 * (isBlue ? 1 : -1), isBlue ? 245 : 295, DEFAULT_POWER, 2, 2500); //pivot platform  //WAS x = 50 //WAS angle = isBlue ? 225 : 315

            moveSCARA(robot.controller.INSIDE_ROBOT_TO_DELIVERY);
            robot.openGrabber();
            //moveTo(110, isBlue ? 140 : -140, 270, DEFAULT_POWER, 5, 2000); //push platform
            moveTo(90 - secondStoneXOffset, isBlue ? 100 : -100, isBlue ? 245 : 295, DEFAULT_POWER, 5, 2000); //push platform

            robot.grabberServo.setPosition(0.5);
            robot.moveLiftToPosition(400); //was 175
            moveSCARA(robot.controller.DELIVERY_TO_INSIDE_ROBOT);
            robot.moveLiftToPosition(0);

            robot.moveIntake(STOP);

            //simplePathFollow.stop(robot);
            latchHooksUp();

            //park (?)
            moveTo(0 - secondStoneXOffset, (secondYDistance - 30) * (isBlue ? 1 : -1), 270, MID_POWER, 5, 3000);
            robot.moveIntake(STOP);

        } else {
            //park
            moveWithIMU(0, isBlue ? 95 : -95, 270, 0.6, 5); //was y = +- 75
        }

        simplePathFollow.stop(robot);
    }

    private void moveTo(double x, double y, double orientation, double speed, double threshold) {
        boolean done = false;
        long startTime = System.currentTimeMillis();
        while (!done && opModeIsActive() && System.currentTimeMillis() - startTime < 6000) { //default 6 second timeout
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
        long startTime = System.currentTimeMillis();
        while (!done && opModeIsActive() && System.currentTimeMillis() - startTime < 6000) { //default 6 second timeout
            robot.updateBulkData();
            simpleTracking.updatePosition(robot, true, isBlue);
            telemetry.addData("Using", "encoders");
            //telemetry.update();
            done = (simplePathFollow.moveToTarget(robot, simpleTracking, x, y, orientation, speed) < threshold);
            logTelemetry();
        }
    }

    private void moveWithIMU(double x, double y, double orientation, double speed, double threshold, double timeout) {
        boolean done = false;
        double startTime = System.currentTimeMillis();
        while (!done && opModeIsActive() && (System.currentTimeMillis() < startTime + timeout)) {
            robot.updateBulkData();
            simpleTracking.updatePosition(robot, true, isBlue);
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
                    (robot.backRangeSensor.getDistance(DistanceUnit.CM) < 2);
            logTelemetry();
        }
    }

    //move SCARA with sequence
    public void moveSCARA (SCARAController.Sequence sequence) {
        double lastTime = getRuntime();
        double currentTime = getRuntime();
        //robot.setPlacerUp();
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

    public void deliverBlock (int deliveryLiftPos) {
        robot.closeGrabber();
        robot.wait(250, this); //was 750
        robot.moveLiftToPosition(deliveryLiftPos); //was 175
        robot.wait(250, this); //was 750
        moveSCARA(robot.controller.INSIDE_ROBOT_TO_DELIVERY);
        robot.openGrabber();
        robot.wait(250, this); //was 500
        robot.grabberServo.setPosition(0.5);
        //robot.setPlacerUp();
        robot.moveLiftToPosition(400); //was 175
        moveSCARA(robot.controller.DELIVERY_TO_INSIDE_ROBOT);
        robot.moveLiftToPosition(0);
    }

    public void prepareToGrab () {
        //robot.setPlacerUp();
        robot.moveGrabberToMid();
        moveSCARA(robot.controller.DELIVERY_TO_INSIDE_ROBOT);
    }

    //move relative to the current position
    private void move(double x, double y, double orientation, double speed, double threshold) {
        moveTo(simpleTracking.lastRobotX + x, simpleTracking.lastRobotY + y, simpleTracking.lastRobotOrientation + orientation, speed, threshold);
    }

    public void updateOrientationWithIMU(boolean isBlue) {
        double angle = robot.getRobotHeading().getAngle();
        if (isBlue) angle += 180;
        if (angle < 0) angle += 360;
        if (angle >= 360) angle -= 360;
        simpleTracking.setOrientationDegrees(angle);
    }

    public void waitForButton() {
        while (!gamepad1.b && opModeIsActive()) {
            simpleTracking.updatePosition(robot);
            telemetry.addData("IMU Heading: ", robot.getRobotHeading());
            logTelemetry();
        }
    }

    public void latchHooksDown () {
        robot.latchServo1.setPosition(0.0);
        robot.latchServo2.setPosition(1.0);
    }

    public void latchHooksUp () {
        robot.latchServo1.setPosition(1.0);
        robot.latchServo2.setPosition(0.0);
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
