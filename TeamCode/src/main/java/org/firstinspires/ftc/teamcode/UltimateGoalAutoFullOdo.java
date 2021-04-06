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
import org.firstinspires.ftc.teamcode.HelperClasses.WayPoint;
import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;

import java.util.List;

import static org.firstinspires.ftc.teamcode.UltimateGoalAutoFullOdo.StartShotStyle.STRAFE_STOP;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.GRABBING.IDLE;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.WOBBLE_ARM_DEPLOYING;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.WOBBLE_ARM_GRABBING;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.WOBBLE_ARM_RUNNING;
import static org.firstinspires.ftc.teamcode.UltimateGoalRobot.WOBBLE_ARM_STOWED;

/**
 * Created by 12090 STEM Punk
 */
public abstract class UltimateGoalAutoFullOdo extends UltimateGoalAutoBase
{
    protected int randomizationPosition = 1;

    enum StartShotStyle {
        STRAFE_STOP,
        ROTATE,
        STRAFE_THROUGH,
        HIGH_GOAL
    }
    StartShotStyle startShootingStyle = StartShotStyle.STRAFE_STOP;

    // This allows us to set waypoints per alliance and potentially position.
    public abstract void setAutoWayPoints();
    // This sets the waypoints for randomization specific after match start.
    public abstract void setRandomizationPosition(int position);

    public void dropWobbleTargetZone() {
        driveToWayPoint(targetZone1, false, true);
        while(opModeIsActive() && robot.armMovement != UltimateGoalRobot.WOBBLE_ARM_ROTATOR.IDLE) {
            updatePosition();
        }
        robot.startClawToggle(true);
        while(opModeIsActive() && robot.grabState != IDLE) {
            updatePosition();
        }
    }

    public void shootPowerShotStrafeStopStyle() {
        robot.shooterOnPowershot();
        driveToWayPoint(aroundStartingStack1, true, true);
        robot.startRotatingArm(WOBBLE_ARM_DEPLOYING);
        driveToWayPoint(powerShotFirst, false, false);
        sleep(500);
        robot.startInjecting();
        while (opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
            updatePosition();
        }
        driveToWayPoint(powerShotSecond, false, false);
        robot.startInjecting();
        while (opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
            updatePosition();
        }
        driveToWayPoint(powerShotThird, false, false);
        robot.startInjecting();
        while (opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
            updatePosition();
        }

        if (randomizationPosition == 1) {
            robot.shooterOff();
        } else {
            robot.shooterOnHighGoal();
        }
    }

    public void shootPowerShotRotateStyle() {
        robot.shooterOnPowershot();
        driveToWayPoint(aroundStartingStack1, true, true);
        robot.startRotatingArm(WOBBLE_ARM_DEPLOYING);
        driveToWayPoint(powerShotFirst, false, false);
        robot.startInjecting();
        while(opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
            updatePosition();
        }
        rotateToWayPointAngle(powerShotSecond);
        robot.startInjecting();
        while(opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
            updatePosition();
        }
        rotateToWayPointAngle(powerShotThird);
        robot.startInjecting();
        while(opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
            updatePosition();
        }

        robot.shooterOff();
    }
    public void shootPowerShotStrafeThroughStyle() {
        robot.shooterOnPowershot();
        driveToWayPoint(aroundStartingStack1, true, true);
        robot.startRotatingArm(WOBBLE_ARM_DEPLOYING);
        driveToWayPoint(powerShotFirst, false, false);
        robot.startInjecting();
        while(opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
            updatePosition();
        }
        driveToWayPoint(powerShotSecond,true, false);
        robot.startInjecting();
        while(opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
            updatePosition();
        }
        driveToWayPoint(powerShotThird, true, false);
        robot.startInjecting();
        while(opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
            updatePosition();
        }

        robot.shooterOff();
    }
    public void shootHighGoal() {
        robot.shooterOnLongShotHighGoal();
        driveToWayPoint(longShotHighGoal, false, false);
        robot.startTripleInjecting();
        while(opModeIsActive() && robot.tripleInjectState != UltimateGoalRobot.TRIPLE_INJECTING.IDLE) {
            updatePosition();
        }
        robot.shooterOnQuadHighGoal();
        robot.startRotatingArm(WOBBLE_ARM_DEPLOYING);
        robot.setIntakeIn();
        driveToWayPoint(beforeQuadStack, true, true);
        driveToWayPoint(collectQuadStack, false, true);
        driveToWayPoint(quadHighGoalCollecting, false, false);
        robot.startInjecting();
        // shoot
        while(opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
            updatePosition();
        }
        robot.shooterOnHighGoal();
        driveToWayPoint(collectQuadStraggler, true, true);
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
            robot.finalAutoPosition = new WayPoint(startLocation.x, startLocation.y, startLocation.angle, 1.0);
            robot.autoExecuted = true;

            switch (startShootingStyle) {
                case STRAFE_STOP:
                    shootPowerShotStrafeStopStyle();
                    break;
                case STRAFE_THROUGH:
                    shootPowerShotStrafeThroughStyle();
                    break;
                case HIGH_GOAL:
                    shootHighGoal();
                    break;
                case ROTATE:
                    shootPowerShotRotateStyle();
                    break;
            }

            dropWobbleTargetZone();

            // This is for position 1 and 2
            if (randomizationPosition != 3) {
                driveToWayPoint(beforeStack, true, true);
                robot.startRotatingArm(WOBBLE_ARM_GRABBING);
                robot.setIntakeIn();

                driveToWayPoint(collectStack, true, true);
            // This is for position 3
            } else {
                driveToWayPoint(quadSecondWobbleStart, true, true);
                robot.startRotatingArm(WOBBLE_ARM_GRABBING);
                robot.setIntakeOff();
            }

            driveToWayPoint(wobble2PickupLineup, true, false);
            driveToWayPoint(wobble2Pickup, false, false);

            robot.startClawToggle(false);
            while (opModeIsActive() && robot.grabState != IDLE) {
                updatePosition();
            }
            robot.startRotatingArm(WOBBLE_ARM_DEPLOYING);
            while (opModeIsActive() && robot.armMovement != UltimateGoalRobot.WOBBLE_ARM_ROTATOR.IDLE) {
                updatePosition();
            }

            if (randomizationPosition == 1) {
                robot.shooterOff();
                driveToWayPoint(highGoal, true, true);
                robot.setIntakeOff();
            } else if (randomizationPosition == 2) {
                driveToWayPoint(highGoal, false, false);
                robot.startInjecting();
                while (opModeIsActive() && robot.injectState != UltimateGoalRobot.INJECTING.IDLE) {
                    updatePosition();
                }
                robot.shooterOff();
                robot.setIntakeOff();
            } else if (randomizationPosition == 3) {
                driveToWayPoint(quadHighGoalFinal, false, false);
                robot.startTripleInjecting();
                while (opModeIsActive() && robot.tripleInjectState != UltimateGoalRobot.TRIPLE_INJECTING.IDLE) {
                    updatePosition();
                }
                robot.shooterOff();
            }
            driveToWayPoint(targetZone2, false, true);
            robot.startClawToggle(true);
            while (opModeIsActive() && robot.grabState != IDLE) {
                updatePosition();
            }

            // When we run out of time, the wobble goal arm keeps rotating and will break
            // your potentiometer.
            driveToWayPoint(park, false, true);
            robot.startClawToggle(false);
            while (opModeIsActive() && robot.grabState != IDLE) {
                updatePosition();
            }
        }

        updatePosition();
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}