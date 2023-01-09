package org.firstinspires.ftc.blackswan;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name = "MMMMMCOLORS")
public class SensorOfTheColor extends LinearOpMode {

    RevColorSensorV3 colorSensorL;
    RevColorSensorV3 colorSensorR;

    public void runOpMode() {

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            colorSensorL = hardwareMap.get(RevColorSensorV3.class, "colorSensorL");
            colorSensorR = hardwareMap.get(RevColorSensorV3.class, "colorSensorR");

//        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
//        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
//        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
//        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");
//
//        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            waitForStart();

            telemetry.addData("leftBlueValue", colorSensorL.blue());
            telemetry.addData("leftRedValue", colorSensorL.red());
            telemetry.addData("leftBlueValue", colorSensorR.blue());
            telemetry.addData("leftRedValue", colorSensorR.red());

            telemetry.update();

        }
    }
}

