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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class DistanceSensorBot extends WobbleGoalBot {

    protected DistanceSensor distSensor = null;

    public double sensorDistance = 0;

    public boolean isRepeating = false;

    boolean shouldGrabDrive = false;
    private boolean grabDriveForward = true;

    private long lastToggleDone6 = 0;
    private long timeSinceToggle6 = 0;

    public DistanceSensorBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        distSensor = hwMap.get(DistanceSensor.class, "distanceSensor");
    }

    public void getDistance() {
//        opMode.telemetry.addData("range", String.format("%.01f cm", distSensor.getDistance(DistanceUnit.CM)));

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
//        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distSensor;
        // Rev2mDistanceSensor specific methods.
//        opMode.telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
//        opMode.telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
//        opMode.telemetry.update();
        sensorDistance = distSensor.getDistance(DistanceUnit.CM);
    }

    public void checkFreightInBox() {
        if (servoPosIndex == 2 && isRepeating && (50 > Math.abs(wobbleArm.getCurrentPosition() - armPositions[0]))) {
            isRepeating = false;
        }
        if (sensorDistance < 3 && !isRepeating && armPosIndex == 0) {
            isRepeating = true;
            servoPosIndex = 1;
            wobblePinch.setPosition(servoPositions[servoPosIndex]);
            inOutPosIndex = 1;
            inOut.setPosition(inOutPositions[inOutPosIndex]);
            isIntakeSpinning = false;
            //sleep(5000);
            controlWobbleArm(true, false);
        }
    }

    public void grabDrive(double power) {
        timeSinceToggle6 = System.currentTimeMillis() - lastToggleDone6;
        if (shouldGrabDrive && grabDriveForward && timeSinceToggle6 > 1200) {
            leftFront.setPower(power);
            rightFront.setPower(power);
            leftRear.setPower(power);
            rightRear.setPower(power);
            grabDriveForward = false;
            lastToggleDone6 = System.currentTimeMillis();
        } else if (shouldGrabDrive && !grabDriveForward && timeSinceToggle6 > 1800) {
            leftFront.setPower(-power);
            rightFront.setPower(-power);
            leftRear.setPower(-power);
            rightRear.setPower(-power);
            grabDriveForward = true;
            lastToggleDone6 = System.currentTimeMillis();
        }
    }

    public void autoGrabFreight(double power) {
        int LFStartingPos = leftFront.getCurrentPosition();
        int RFStartingPos = rightFront.getCurrentPosition();
        int LRStartingPos = leftRear.getCurrentPosition();
        int RRStartingPos = rightRear.getCurrentPosition();

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        goToInOutPosition(1);
        isIntakeSpinning = true;
//        while (sensorDistance > 14 && opMode.opModeIsActive()) {
//            onLoop(50, "autoGrab");
//        }
//        leftFront.setPower(0);
//        rightFront.setPower(0);
//        leftRear.setPower(0);
//        rightRear.setPower(0);
//        inOutPosIndex = 0;
//        inOut.setPosition(inOutPositions[inOutPosIndex]);
        //int distanceTravelled = Math.abs(leftFront.getCurrentPosition() - LFStartingPos);
        shouldToggle = true;
        shouldGrabDrive = true;
        long autoGrabStart = System.currentTimeMillis();
        long timeSinceAutoGrab = 0;
        while (sensorDistance > 3 && timeSinceAutoGrab < 5000 && opMode.opModeIsActive() ){
            isRepeating = true;
            onLoop(50, "autoGrab 2");
            timeSinceAutoGrab = Math.abs(autoGrabStart - System.currentTimeMillis());
            opMode.telemetry.addData("time: ", timeSinceAutoGrab);
            opMode.telemetry.update();
        }
        goToInOutPosition(1);
        shouldToggle = false;
        shouldGrabDrive = false;
        servoPosIndex = 1;
        wobblePinch.setPosition(servoPositions[servoPosIndex]);

        isIntakeSpinning = false;
        shouldUpdateIntake = false;
        intake.setPower(-1);
        leftFront.setTargetPosition(LFStartingPos);
        rightFront.setTargetPosition(RFStartingPos);
        leftRear.setTargetPosition(LRStartingPos);
        rightRear.setTargetPosition(RRStartingPos);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(0.3);
        rightFront.setPower(0.3);
        leftRear.setPower(0.3);
        rightRear.setPower(0.3);
        setArmPositionNoWait(-600, 0.18);
        while (opMode.opModeIsActive() && rightFront.isBusy()) {
            onLoop(50, "Driving straight by distance");
        }
        intake.setPower(0);
        shouldUpdateIntake = true;
        grabDriveForward = true;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    protected void onTick() {
        getDistance();
        checkFreightInBox();
        grabDrive(0.12);
//        opMode.telemetry.addData("distance: ", sensorDistance);
//        opMode.telemetry.addData("isRepeating: ", isRepeating);
//        opMode.telemetry.addData("isSpinning: ", isIntakeSpinning);
//        opMode.telemetry.addData("wobbleArm position: ", Math.abs(wobbleArm.getCurrentPosition() - armPositions[0]));
//        opMode.telemetry.update();
        super.onTick();
    }
}
