package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Devices {
        final double CLAW_OPEN = .864;
        final double CLAW_CLOSED = 1.05;

        final double MIN_SLIDER_TICKS = 10; // chosen just to be safe
        final double MAX_SLIDER_TICKS = 2735.4;
        final double MAX_SLIDER_INCHES = 24.25;

        DcMotorEx wormGear;
        DcMotorEx sliderMotor;
        DcMotorEx leftFrontDrive;
        DcMotorEx rightFrontDrive;
        DcMotorEx rightBackDrive;
        DcMotorEx leftBackDrive;
        Servo clawServo;

    public void init(HardwareMap hwmap) {

        wormGear = hwmap.get(DcMotorEx.class, "wormGear");

        sliderMotor = hwmap.get(DcMotorEx.class, "sliderMotor");

        leftFrontDrive = hwmap.get(DcMotorEx.class, "leftFront");
        rightFrontDrive = hwmap.get(DcMotorEx.class, "rightFront");
        leftBackDrive = hwmap.get(DcMotorEx.class, "leftBack");
        rightBackDrive = hwmap.get(DcMotorEx.class, "rightBack");

        clawServo = hwmap.get(Servo.class, "clawServo");

        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
