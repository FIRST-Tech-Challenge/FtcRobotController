/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;

import java.util.List;

import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.GRABBING.IDLE;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.WOBBLE_ARM_DEPLOYING;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.WOBBLE_ARM_RUNNING;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.WOBBLE_ARM_STOWED;

/**
 * Created by 12090 STEM Punk
 */
public abstract class UltimateGoalAutoFullOdo extends UltimateGoalAutoBase
{
    protected int randomizationPosition = 1;

    // This allows us to set waypoints per alliance and potentially position.
    public abstract void setAutoWayPoints();
    // This sets the waypoints for randomization specific after match start.
    public abstract void setRandomizationPosition(int position);

    public void shootHighGoal(){
    }

    public void dropWobbleTargetZone() {
        driveToWayPoint(targetZone1, false);
        robot.startRotatingArm(WOBBLE_ARM_DEPLOYING);
        while(opModeIsActive() && robot.armMovement != UltimateGoalRobot.WOBBLE_ARM_ROTATOR.IDLE) {
            updatePosition();
        }
        robot.startClawToggle(true);
        while(opModeIsActive() && robot.grabState != IDLE) {
            updatePosition();
        }
    }
    public void dropWobbleTargetZoneA() {
        dropWobbleTargetZone();
    }

    public void dropWobbleTargetZoneB() {
        dropWobbleTargetZone();
    }

    public void dropWobbleTargetZoneC() {
        dropWobbleTargetZone();
    }

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.5, 16.0/9.0);
        }

        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        setupRobotParameters();

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
        robot.encodersReset = true;

        setAutoWayPoints();
        /*
         * Wait for the user to press start on the Driver Station
         */
        while(!isStarted()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.addData("Randomization Position: ", randomizationPosition);
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                            randomizationPosition = 3;
                        } else {
                            randomizationPosition = 2;
                        }
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                    telemetry.update();
                }
            }
        }

        // In case Stop was pressed.
        if(opModeIsActive()) {
            autoTimer.reset();
            telemetry.addData("Target Zone: ", randomizationPosition);
            telemetry.update();

            // This sets up everything for the auto to run.
            setRandomizationPosition(randomizationPosition);

            //give MyPosition our current positions so that it saves the last positions of the wheels
            //this means we won't teleport when we start the match. Just in case, run this twice
            for (int i = 0; i < 2; i++) {
                robot.resetReads();
                MyPosition.initialize(robot.getLeftEncoderWheelPosition(),
                        robot.getRightEncoderWheelPosition(),
                        robot.getStrafeEncoderWheelPosition());
            }

            // Set our robot starting coordinates on the field.
            robot.resetReads();
            MyPosition.setPosition(startLocation.x, startLocation.y, startLocation.angle);

            robot.setShooterFlapPowerShot();
            driveToWayPoint(aroundStartingStack1, true);
            driveToWayPoint(powerShotFirst, false);
            robot.startInjecting();
            while(opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
                updatePosition();
            }
            driveToWayPoint(powerShotSecond,false);
            robot.startInjecting();
            while(opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
                updatePosition();
            }
            driveToWayPoint(powerShotThird, false);
            robot.startInjecting();
            while(opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
                updatePosition();
            }

            robot.shooterOff();

            dropWobbleTargetZone();

            driveToWayPoint(park, false);
            robot.startRotatingArm(WOBBLE_ARM_STOWED);
            while(opModeIsActive() && robot.armMovement != UltimateGoalRobot.WOBBLE_ARM_ROTATOR.IDLE) {
                updatePosition();
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * @param destinationAngle - The target angle to reach, between 0.0 and 360.0
     * @param gyroReading      - The current angle of the robot
     * @return The minumum angle to travel to get to the destination angle
     */
    private double deltaAngle(double destinationAngle, double gyroReading) {
        double result = 0.0;
        double leftResult = 0.0;
        double rightResult = 0.0;

        if (gyroReading > destinationAngle) {
            leftResult = gyroReading - destinationAngle;
            rightResult = 360.0 - gyroReading + destinationAngle;
        } else {
            leftResult = gyroReading + 360.0 - destinationAngle;
            rightResult = destinationAngle - gyroReading;
        }

        if (leftResult < rightResult) {
            result = -leftResult;
        } else {
            result = rightResult;
        }

        return result;
    }

    /**
     * @param speed        - The driving power
     * @param rotateSpeed  - The rotational speed to correct heading errors
     * @param driveAngle   - The angle of movement to drive the robot
     * @param headingAngle - The heading angle to hold while driving
     */
    public void driveAtHeading(double speed, double rotateSpeed, double driveAngle, double headingAngle) {
        double xPower = 0.0;
        double yPower = 0.0;
        double deltaAngle = 0.0;
        final double SAME_ANGLE = 1;
        double gyroReading = robot.readIMU();
        deltaAngle = deltaAngle(headingAngle, gyroReading);

        if (Math.abs(deltaAngle) > SAME_ANGLE) {
            if (deltaAngle > 0.0) {
                rotateSpeed = -rotateSpeed;
            }
        } else {
            rotateSpeed = 0.0;
        }

        xPower = speed * Math.cos(Math.toRadians(driveAngle));
        yPower = speed * Math.sin(Math.toRadians(driveAngle));
        robot.drive(xPower, yPower, rotateSpeed, 0.0, false);
    }

    /**
     * @param speed        - The driving power
     * @param rotateSpeed  - The rotational speed to correct heading errors
     * @param driveAngle   - The angle of movement to drive the robot
     * @param headingAngle - The heading angle to hold while driving
     */
    public void driveAtHeadingForTime(double speed, double rotateSpeed, double driveAngle, double headingAngle, int driveTime, boolean stopWhenDone, boolean progressActivities) {
        double endTime = timer.milliseconds() + driveTime;
        while (!isStopRequested() && (timer.milliseconds() <= endTime) && (!isStopRequested())) {
            robot.resetReads();
            driveAtHeading(speed, rotateSpeed, driveAngle, headingAngle);
            if(progressActivities) {
                performRobotActions();
            }
        }
        if(stopWhenDone) {
            robot.setAllDriveZero();
        }
    }
}