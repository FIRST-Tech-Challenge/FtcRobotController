package org.firstinspires.ftc.masters.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.masters.CSCons;

public class TouchSensorAdjustment extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Servo outtakeAngle = hardwareMap.servo.get("outtakeAngle");
        TouchSensor sensor = hardwareMap.touchSensor.get("touchBucket");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            outtakeAngle.setPosition(CSCons.outtakeAngleTransfer);


        }
    }
}
