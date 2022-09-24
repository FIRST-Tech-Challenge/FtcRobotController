/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public class FSMBot extends NewDistanceSensorBot {

    ElapsedTime snarmTimer = new ElapsedTime();
    ElapsedTime timeSince1 = new ElapsedTime(500);
    ElapsedTime timeSince2 = new ElapsedTime(500);
    ElapsedTime timeSince3 = new ElapsedTime(500);
    ElapsedTime timeSince4 = new ElapsedTime(500);
    ElapsedTime timeSince5 = new ElapsedTime(500);
    ElapsedTime timeSince6 = new ElapsedTime(500);

    public boolean isAutoStart = false;
    public boolean shouldAutoExtend = true;
    public boolean drivingDone = false;
    public boolean shouldIdle = false;
    public boolean breakOutOfIdle = false;
    public boolean isAllianceHub = true;
    public boolean manualIntakeSensor = false;
    public boolean bypassAfterReadyAgain = true;

    public int snarmSnarmState = 0;

    protected double dropHeight = 0;
    protected double snarmRotation = rotationCenter;
    protected double retractionStep = 2600;
    protected double extensionStep1 = 200;
    protected double extensionStep2 = 1000;
    final public double snarmIntakingHeight = 0.04;
    final protected double snarmTravelHeight = 0.3;

    final public double blueAllianceRotation = 0.66; //0.6
    final public double redAllianceRotation = 0.34; //0.25
    final public double blueSharedRotation = 0.7; //0.63
    final public double redSharedRotation = 0.29; //0.22

    public FSMBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        snarmTimer.reset();
        drivingDone = false;
        if (!isAutonomous) {
            snarmState = SnarmState.AFTER_READY_AGAIN;
        }
    }

    public void setDropHeight(int pos) {
        switch (pos) {
            case 0:
                dropHeight = 0.25;
                maxExtension = 3100;
                extensionStep1 = 200;
                extensionStep2 = 1000;
                retractionStep = 2600;
                break;
            case 1:
                dropHeight = 0.46;
                maxExtension = 3100;
                extensionStep1 = 200;
                extensionStep2 = 1000;
                retractionStep = 2600;
                break;
            case 2:
                dropHeight = 0.68;
                maxExtension = 2900;
                extensionStep1 = 200;
                extensionStep2 = 1000;
                retractionStep = 2600;
                break;
            case 3:
                dropHeight = 0.9;
                maxExtension = 1000;
                extensionStep1 = 200;
                extensionStep2 = 500;
                retractionStep = 300;
                break;
            case 4:
                dropHeight = 0.22;
                maxExtension = 1500;
                extensionStep1 = 200;
                extensionStep2 = 600;
                retractionStep = 1000;
                break;
        }
    }

    public void setSnarmRotation(int side) {
        switch (side) {
            case 0:
                snarmRotation = blueAllianceRotation; //0.6
                break;
            case 1:
                snarmRotation = redAllianceRotation; // 0.25
                break;
            case 2:
                snarmRotation = blueSharedRotation; // 0.63
                break;
            case 3:
                snarmRotation = redSharedRotation; // 0.22
                break;
            case 4:
                snarmRotation = blueAllianceRotation;
        }
    }

    public void toggleHub(boolean blue, boolean button) {
        if (button && blue && timeSince5.milliseconds() > 500) {
            if (isAllianceHub) {
                setSnarmRotation(3);
                setDropHeight(3);
                isAllianceHub = false;
                snarmSnarmState = 2;
                opMode.telemetry.addData("switched to shared", true);
            } else {
                setSnarmRotation(0);
                setDropHeight(0);
                isAllianceHub = true;
                snarmSnarmState = 1;
                opMode.telemetry.addData("switched to alliance", true);
            }
            opMode.telemetry.update();
            timeSince5.reset();
        } else if (button && timeSince5.milliseconds() > 500) {
            if (isAllianceHub) {
                setSnarmRotation(2);
                setDropHeight(3);
                isAllianceHub = false;
                snarmSnarmState = 2;
                opMode.telemetry.addData("switched to shared", true);
            } else {
                setSnarmRotation(1);
                setDropHeight(0);
                isAllianceHub = true;
                snarmSnarmState = 1;
                opMode.telemetry.addData("switched to alliance", true);
            }
            opMode.telemetry.update();
            timeSince5.reset();
        }
    }

    public void autoExtend(boolean button) {
        if (button && timeSince1.milliseconds() > 500) {
            snarmState = SnarmState.READY;
            isAutoStart = true;
            shouldAutoExtend = true;
            timeSince1.reset();
        } else {
            shouldAutoExtend = false;
        }
    }

    public void autoRetract(boolean button) {
        if (button && timeSince2.milliseconds() > 500) {
            setExtension(minExtension);
            snarmState = SnarmState.RETRACTING_STAGE_1;
            timeSince2.reset();
        }
    }

    public void dropFreight(boolean button) {
        if (button && timeSince3.milliseconds() > 500) {
            snarmState = SnarmState.EXTENDING_STAGE_3;
            //extender.setTargetPosition(extender.getCurrentPosition());
            isAutonomous = true;
            timeSince3.reset();
        } else {
            isAutonomous = false;
        }
    }

    public void prepareTape(boolean button) {
        if (button && timeSince5.milliseconds() > 500) {
            snarmState = SnarmState.IDLE;

            timeSince5.reset();
        }
    }

    public void toggleBox(boolean button) {
        if (button && timeSince4.milliseconds() > 500) {
            if (box.getPosition() == boxOpened) {
                box.setPosition(boxLocked);
                timeSince4.reset();
            } else if (box.getPosition() == boxLocked || box.getPosition() == boxInit){
                box.setPosition(boxOpened);
                timeSince4.reset();
            }
        }
    }

    public void manualRaiseIntake(boolean button) {
        manualIntakeSensor = button;
    }

    protected void onTick() {
        switch (snarmSnarmState) {
            case 0:
                switch (snarmState) {
                    case READY:
                        if (isAutoStart) {
                            RobotLog.d("AUTO: ready");

                            setElevationPosition(elevationInit);
                            setRotationPosition(rotationInit);
                            setExtension(maxExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(0);

                            stopRotation();
                            goToIntakePosition(3);

                            isAutoStart = false;

                            snarmState = SnarmState.EXTENDING_STAGE_1;
                        }
                        break;
                    case EXTENDING_STAGE_1:
                        if (extender.getCurrentPosition() > extensionStep1) {
                            RobotLog.d("AUTO: 200 passed");

                            setElevationPosition(dropHeight);
                            setRotationPosition(snarmRotation);
                            setExtension(maxExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(0);

                            stopRotation();
                            //goToIntakePosition(3);

                            snarmState = SnarmState.EXTENDING_STAGE_2;
                        }
                        break;
                    case EXTENDING_STAGE_2:
                        if (extender.getCurrentPosition() > extensionStep2) {
                            RobotLog.d("AUTO: 1000 passed");

                            setElevationPosition(dropHeight);
                            setRotationPosition(snarmRotation);
                            setExtension(maxExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(3);

                            stopRotation();
                            goToIntakePosition(5);

                            snarmState = SnarmState.EXTENDING_STAGE_3;
                        }
                        break;
                    case EXTENDING_STAGE_3:
                        if (extender.getCurrentPosition() > maxExtension - 100) {
                            RobotLog.d("AUTO: max passed");

                            setElevationPosition(dropHeight);
                            setRotationPosition(snarmRotation);
                            setExtension(maxExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(3);

                            stopRotation();
                            //goToIntakePosition(3);

                            snarmTimer.reset();

                            snarmState = SnarmState.RELEASING;
                        }
                        break;
                    case RELEASING:
                        if (snarmTimer.milliseconds() >= 100) {
                            RobotLog.d("AUTO: retraction started");

                            setElevationPosition(dropHeight);
                            setRotationPosition(rotationAvoid);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            stopRotation();
                            goToIntakePosition(3);

                            snarmState = SnarmState.RETRACTING_STAGE_1;
                        }
                        break;
                    case RETRACTING_STAGE_1:
                        if (extender.getCurrentPosition() < 2600) {
                            RobotLog.d("AUTO: 2600 passed");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationAvoid);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            startRotation();
                            goToIntakePosition(3);

                            snarmState = SnarmState.INTAKING;
                        }
                        break;
                    case INTAKING:
                        if (drivingDone && (extender.getCurrentPosition() < minExtension + 100)) {
                            RobotLog.d("AUTO: intaking started");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            startRotation();
                            goToIntakePosition(4);

                            drivingDone = false;
                            intakeFast = false;

                            snarmTimer.reset();

                            snarmState = SnarmState.RAISING_INTAKE;
                        }
                        break;
                    case RAISING_INTAKE:
                        if ((distanceIntake < 7 || distanceIntake > 100 || snarmTimer.milliseconds() > 5000) && (intakePosIndex == 3 || intakePosIndex == 4) && extender.getCurrentPosition() < minExtension + 100) {
                            RobotLog.d("AUTO: intake raised");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            stopRotation();
                            goToIntakePosition(2);

                            drivingDone = false;
                            intakeFast = false;

                            leftFront.setPower(0);
                            rightFront.setPower(0);
                            leftRear.setPower(0);
                            rightRear.setPower(0);

                            snarmTimer.reset();

                            snarmState = SnarmState.FEEDING;
                        }
                        break;
                    case FEEDING:
                        if (snarmTimer.milliseconds() >= 500) {
                            RobotLog.d("AUTO: feeding started");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            intakeFast = true;
                            startRotation();
                            //goToIntakePosition(2);

                            snarmTimer.reset();

                            snarmState = SnarmState.READY_AGAIN;
                        }
                        break;
                    case READY_AGAIN:
                        if (distanceBox < 8 || snarmTimer.milliseconds() > 1300) {
                            RobotLog.d("AUTO: ready again");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(0);

                            intakeFast = true;
                            stopRotation();
                            goToIntakePosition(2);

                            isAutoStart = false;

                            snarmState = SnarmState.AFTER_READY_AGAIN;
                        }
                        break;
                    case AFTER_READY_AGAIN:
                        if (snarmTimer.milliseconds() > 150 && keepExtending) {
                            RobotLog.d("AUTO: after ready again");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(maxExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(0);

                            intakeFast = false;
                            stopRotation();
                            goToIntakePosition(3);

                            isAutoStart = false;

                            snarmState = SnarmState.EXTENDING_STAGE_1;
                        }
                        break;
                    case IDLE:
                        if (!shouldIdle) {
                            RobotLog.d("AUTO: idle");

                            setElevationPosition(0.2);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(4);

                            intakeFast = false;
                            stopRotation();
                            goToIntakePosition(0);

                            isAutoStart = false;

                            snarmState = SnarmState.READY;
                        }
                        break;
                    case IDLE_WAIT:
                        if (breakOutOfIdle) {
                            RobotLog.d("AUTO: idle_wait");

                            setElevationPosition(0.1);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(0);

                            intakeFast = false;
                            stopRotation();
                            goToIntakePosition(3);

                            isAutoStart = false;

                            snarmState = SnarmState.EXTENDING_STAGE_1;
                        }
                        break;
                }
                break;
            case 1:
                switch (snarmState) {
                    case READY:
                        if (isAutoStart) {
                            RobotLog.d("ALLIANCE: ready");

                            setElevationPosition(elevationInit);
                            setRotationPosition(rotationInit);
                            setExtension(maxExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(0);

                            stopRotation();
                            goToIntakePosition(3);

                            isAutoStart = false;

                            snarmState = SnarmState.EXTENDING_STAGE_1;
                        }
                        break;
                    case EXTENDING_STAGE_1:
                        if (extender.getCurrentPosition() > extensionStep1) {
                            RobotLog.d("ALLIANCE: 200 passed");

                            setElevationPosition(dropHeight);
                            setRotationPosition(snarmRotation);
                            setExtension(maxExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(0);

                            stopRotation();
                            //goToIntakePosition(3);

                            snarmState = SnarmState.EXTENDING_STAGE_2;
                        }
                        break;
                    case EXTENDING_STAGE_2:
                        if (extender.getCurrentPosition() > extensionStep2) {
                            RobotLog.d("ALLIANCE: 1000 passed");

                            setElevationPosition(dropHeight);
                            setRotationPosition(snarmRotation);
                            setExtension(maxExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(3);

                            stopRotation();
                            goToIntakePosition(5);

                            snarmState = SnarmState.EXTENDING_STAGE_3;
                        }
                        break;
                    case EXTENDING_STAGE_3:
                        if (isAutonomous) {
                            RobotLog.d("ALLIANCE: max passed");

                            //setElevationPosition(dropHeight);
                            //setRotationPosition(snarmRotation);
                            //setExtension(maxExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(3);

                            stopRotation();
                            //goToIntakePosition(3);

                            snarmTimer.reset();

                            snarmState = SnarmState.RELEASING;
                        }
                        break;
                    case RELEASING:
                        if (snarmTimer.milliseconds() >= 150) {
                            RobotLog.d("ALLIANCE: retraction started");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationAvoid);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(3);

                            stopRotation();
                            goToIntakePosition(3);

                            snarmState = SnarmState.RETRACTING_STAGE_1;
                        }
                        break;
                    case RETRACTING_STAGE_1:
                        if (extender.getCurrentPosition() < retractionStep) {
                            RobotLog.d("ALLIANCE: 2600 passed");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationAvoid);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(3);

                            stopRotation();
                            //goToIntakePosition(3);

                            snarmState = SnarmState.INTAKING;
                        }
                        break;
                    case INTAKING:
                        if ((extender.getCurrentPosition() < minExtension + 100)) {
                            RobotLog.d("ALLIANCE: intaking started");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationAvoid);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            startRotation();
                            goToIntakePosition(4);

                            drivingDone = false;
                            intakeFast = false;

                            snarmState = SnarmState.RAISING_INTAKE;
                        }
                        break;
                    case RAISING_INTAKE:
                        if ((distanceIntake < 5 || manualIntakeSensor) && (intakePosIndex == 3 || intakePosIndex == 4) && extender.getCurrentPosition() < minExtension + 100 && !shouldTimeSpin) {
                            RobotLog.d("ALLIANCE: intake raised");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            stopRotation();
                            goToIntakePosition(2);

                            drivingDone = false;
                            intakeFast = false;

                            leftFront.setPower(0);
                            rightFront.setPower(0);
                            leftRear.setPower(0);
                            rightRear.setPower(0);

                            snarmTimer.reset();

                            snarmState = SnarmState.FEEDING;
                        }
                        break;
                    case FEEDING:
                        if (snarmTimer.milliseconds() >= 500) {
                            RobotLog.d("ALLIANCE: feeding started");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            intakeFast = true;
                            startRotation();
                            //goToIntakePosition(2);

                            snarmTimer.reset();

                            snarmState = SnarmState.READY_AGAIN;
                        }
                        break;
                    case READY_AGAIN:
                        if (distanceBox < 8 || snarmTimer.milliseconds() > 1000) {
                            RobotLog.d("ALLIANCE: ready again");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            intakeFast = true;
                            startRotation();
                            goToIntakePosition(2);

                            isAutoStart = false;

                            snarmTimer.reset();

                            snarmState = SnarmState.AFTER_READY_AGAIN;
                        }
                        break;
                    case AFTER_READY_AGAIN:
                        if (snarmTimer.milliseconds() > 150 && bypassAfterReadyAgain) {
                            RobotLog.d("ALLIANCE: after ready again");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(0);

                            intakeFast = false;
                            stopRotation();
                            goToIntakePosition(2);

                            isAutoStart = false;

                            snarmTimer.reset();

                            snarmState = SnarmState.READY;
                        }
                        break;
                    case IDLE:
                        if (!shouldIdle) {
                            RobotLog.d("ALLIANCE: idle");

                            setElevationPosition(0.2);
                            //setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(5);

                            intakeFast = false;
                            stopRotation();
                            goToIntakePosition(3);

                            isAutoStart = false;

                            snarmState = SnarmState.READY;
                        }
                        break;
                }
            case 2:
                switch (snarmState) {
                    case READY:
                        if (isAutoStart) {
                            RobotLog.d("SHARED: ready");

                            setElevationPosition(elevationInit);
                            setRotationPosition(rotationInit);
                            setExtension(maxExtension - 200);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(0);

                            stopRotation();
                            goToIntakePosition(3);

                            isAutoStart = false;

                            snarmState = SnarmState.EXTENDING_STAGE_1;
                        }
                        break;
                    case EXTENDING_STAGE_1:
                        if (extender.getCurrentPosition() > extensionStep1) {
                            RobotLog.d("SHARED: 200 passed");

                            setElevationPosition(dropHeight);
                            setRotationPosition(snarmRotation);
                            setExtension(maxExtension - 200);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(0);

                            stopRotation();
                            //goToIntakePosition(3);

                            snarmState = SnarmState.EXTENDING_STAGE_2;
                        }
                        break;
                    case EXTENDING_STAGE_2:
                        if (extender.getCurrentPosition() > extensionStep2) {
                            RobotLog.d("SHARED: 1000 passed");

                            setElevationPosition(dropHeight);
                            setRotationPosition(snarmRotation);
                            setExtension(maxExtension - 200);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(3);

                            stopRotation();
                            goToIntakePosition(5);

                            snarmState = SnarmState.EXTENDING_STAGE_3;
                        }
                        break;
                    case EXTENDING_STAGE_3:
                        if (isAutonomous) {
                            RobotLog.d("SHARED: max passed");

                            //setElevationPosition(dropHeight);
                            //setRotationPosition(snarmRotation);
                            //setExtension(maxExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(3);

                            stopRotation();
                            //goToIntakePosition(3);

                            snarmTimer.reset();

                            snarmState = SnarmState.RELEASING;
                        }
                        break;
                    case RELEASING:
                        if (snarmTimer.milliseconds() >= 300) {
                            RobotLog.d("SHARED: retraction started");

                            setElevationPosition(snarmTravelHeight);
                            //setRotationPosition(snarmRotation);
                            //setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(5);

                            stopRotation();
                            goToIntakePosition(3);

                            snarmState = SnarmState.RETRACTING_STAGE_1;
                        }
                        break;
                    case RETRACTING_STAGE_1:
                        if (snarmTimer.milliseconds() >= 150) {
                            RobotLog.d("SHARED: 2600 passed");

                            setElevationPosition(snarmTravelHeight);
                            //setRotationPosition(rotationCenter);
                            setExtension(minExtension, 0.5);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(5);

                            stopRotation();
                            //goToIntakePosition(3);

                            snarmState = SnarmState.RETRACTING_STAGE_2;
                        }
                        break;
                    case RETRACTING_STAGE_2:
                        if (extender.getCurrentPosition() < retractionStep) {
                            RobotLog.d("SHARED: 2600 passed");

                            setElevationPosition(snarmTravelHeight);
                            setRotationPosition(rotationAvoid);
                            setExtension(minExtension, 0.7);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(5);

                            stopRotation();
                            //goToIntakePosition(3);

                            snarmState = SnarmState.INTAKING;
                        }
                        break;
                    case INTAKING:
                        if ((extender.getCurrentPosition() < minExtension + 100)) {
                            RobotLog.d("SHARED: intaking started");

                            setElevationPosition(snarmTravelHeight);
                            setRotationPosition(rotationAvoid);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(5);

                            startRotation();
                            goToIntakePosition(4);

                            snarmTimer.reset();

                            drivingDone = false;
                            intakeFast = false;

                            snarmState = SnarmState.WAITING_FOR_ROTATION;
                        }
                        break;
                    case WAITING_FOR_ROTATION:
                        if (snarmTimer.milliseconds() >= 1000) {
                            RobotLog.d("SHARED: rotation centered");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationAvoid);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            startRotation();
                            goToIntakePosition(4);

                            drivingDone = false;
                            intakeFast = false;

                            snarmState = SnarmState.RAISING_INTAKE;
                        }
                        break;
                    case RAISING_INTAKE:
                        if ((distanceIntake < 5 || manualIntakeSensor) && (intakePosIndex == 3 || intakePosIndex == 4) && extender.getCurrentPosition() < minExtension + 100 && !shouldTimeSpin) {
                            RobotLog.d("SHARED: intake raised");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            stopRotation();
                            goToIntakePosition(2);

                            drivingDone = false;
                            intakeFast = false;

                            leftFront.setPower(0);
                            rightFront.setPower(0);
                            leftRear.setPower(0);
                            rightRear.setPower(0);

                            snarmTimer.reset();

                            snarmState = SnarmState.FEEDING;
                        }
                        break;
                    case FEEDING:
                        if (snarmTimer.milliseconds() >= 500) {
                            RobotLog.d("SHARED: feeding started");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            intakeFast = true;
                            startRotation();
                            //goToIntakePosition(2);

                            snarmTimer.reset();

                            snarmState = SnarmState.READY_AGAIN;
                        }
                        break;
                    case READY_AGAIN:
                        if (distanceBox < 8 || snarmTimer.milliseconds() > 1000) {
                            RobotLog.d("SHARED: ready again");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            intakeFast = true;
                            startRotation();
                            goToIntakePosition(2);

                            isAutoStart = false;

                            snarmTimer.reset();

                            snarmState = SnarmState.AFTER_READY_AGAIN;
                        }
                        break;
                    case AFTER_READY_AGAIN:
                        if (snarmTimer.milliseconds() > 150 && bypassAfterReadyAgain) {
                            RobotLog.d("SHARED: after ready again");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(0);

                            intakeFast = false;
                            stopRotation();
                            goToIntakePosition(2);

                            isAutoStart = false;

                            snarmTimer.reset();

                            snarmState = SnarmState.READY;
                        }
                        break;
                    case IDLE:
                        if (!shouldIdle) {
                            RobotLog.d("SHARED: idle");

                            setElevationPosition(0.2);
                            //setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(5);

                            intakeFast = false;
                            stopRotation();
                            goToIntakePosition(3);

                            isAutoStart = false;

                            snarmState = SnarmState.READY;
                        }
                        break;
                }
            case 3:
                switch (snarmState) {
                    case READY:
                        if (isAutoStart) {
                            RobotLog.d("DEMO: ready");

                            setElevationPosition(elevationInit);
                            setRotationPosition(rotationInit);
                            setExtension(maxExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(0);

                            stopRotation();
                            goToIntakePosition(3);

                            isAutoStart = false;

                            snarmState = SnarmState.EXTENDING_STAGE_1;
                        }
                        break;
                    case EXTENDING_STAGE_1:
                        if (extender.getCurrentPosition() > extensionStep1) {
                            RobotLog.d("DEMO: 200 passed");

                            setElevationPosition(dropHeight);
                            setRotationPosition(snarmRotation);
                            setExtension(maxExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(3);

                            stopRotation();
                            //goToIntakePosition(3);

                            snarmState = SnarmState.EXTENDING_STAGE_2;
                        }
                        break;
                    case EXTENDING_STAGE_2:
                        if (extender.getCurrentPosition() > extensionStep2) {
                            RobotLog.d("DEMO: 1000 passed");

                            setElevationPosition(dropHeight);
                            setRotationPosition(snarmRotation);
                            setExtension(maxExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(3);

                            stopRotation();
                            goToIntakePosition(5);

                            snarmState = SnarmState.EXTENDING_STAGE_3;
                        }
                        break;
                    case EXTENDING_STAGE_3:
                        if (isAutonomous) {
                            RobotLog.d("DEMO: max passed");

                            //setElevationPosition(dropHeight);
                            //setRotationPosition(snarmRotation);
                            //setExtension(maxExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(3);

                            stopRotation();
                            //goToIntakePosition(3);

                            snarmTimer.reset();

                            snarmState = SnarmState.RELEASING;
                        }
                        break;
                    case RELEASING:
                        if (snarmTimer.milliseconds() >= 150) {
                            RobotLog.d("DEMO: retraction started");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationAvoid);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(3);

                            stopRotation();
                            goToIntakePosition(3);

                            snarmState = SnarmState.RETRACTING_STAGE_1;
                        }
                        break;
                    case RETRACTING_STAGE_1:
                        if (extender.getCurrentPosition() < retractionStep) {
                            RobotLog.d("DEMO: 2600 passed");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationAvoid);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(3);

                            stopRotation();
                            //goToIntakePosition(3);

                            snarmState = SnarmState.INTAKING;
                        }
                        break;
                    case INTAKING:
                        if ((extender.getCurrentPosition() < minExtension + 100)) {
                            RobotLog.d("DEMO: intaking started");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationAvoid);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            startRotation();
                            goToIntakePosition(4);

                            drivingDone = false;
                            intakeFast = false;

                            snarmState = SnarmState.RAISING_INTAKE;
                        }
                        break;
                    case RAISING_INTAKE:
                        if ((distanceIntake < 5 || manualIntakeSensor) && (intakePosIndex == 3 || intakePosIndex == 4) && extender.getCurrentPosition() < minExtension + 100 && !shouldTimeSpin) {
                            RobotLog.d("DEMO: intake raised");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            stopRotation();
                            goToIntakePosition(2);

                            drivingDone = false;
                            intakeFast = false;

                            leftFront.setPower(0);
                            rightFront.setPower(0);
                            leftRear.setPower(0);
                            rightRear.setPower(0);

                            snarmTimer.reset();

                            snarmState = SnarmState.FEEDING;
                        }
                        break;
                    case FEEDING:
                        if (snarmTimer.milliseconds() >= 500) {
                            RobotLog.d("DEMO: feeding started");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            intakeFast = true;
                            startRotation();
                            //goToIntakePosition(2);

                            snarmTimer.reset();

                            snarmState = SnarmState.READY_AGAIN;
                        }
                        break;
                    case READY_AGAIN:
                        if (distanceBox < 8 || snarmTimer.milliseconds() > 1000) {
                            RobotLog.d("DEMO: ready again");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxOpened);
                            goToFlipperPosition(0);

                            intakeFast = true;
                            startRotation();
                            goToIntakePosition(2);

                            isAutoStart = false;

                            snarmTimer.reset();

                            snarmState = SnarmState.AFTER_READY_AGAIN;
                        }
                        break;
                    case AFTER_READY_AGAIN:
                        if (snarmTimer.milliseconds() > 150 && bypassAfterReadyAgain) {
                            RobotLog.d("DEMO: after ready again");

                            setElevationPosition(snarmIntakingHeight);
                            setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(0);

                            intakeFast = false;
                            stopRotation();
                            goToIntakePosition(2);

                            isAutoStart = false;

                            snarmTimer.reset();

                            snarmState = SnarmState.READY;
                        }
                        break;
                    case IDLE:
                        if (!shouldIdle) {
                            RobotLog.d("DEMO: idle");

                            setElevationPosition(0.2);
                            //setRotationPosition(rotationCenter);
                            setExtension(minExtension);

                            box.setPosition(boxLocked);
                            goToFlipperPosition(5);

                            intakeFast = false;
                            stopRotation();
                            goToIntakePosition(3);

                            isAutoStart = false;

                            snarmState = SnarmState.READY;
                        }
                        break;
                }
        }
        super.onTick();
    }
}
