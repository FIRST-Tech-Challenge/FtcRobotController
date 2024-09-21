package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class mekanism {
    LinearOpMode myOp;

    DcMotor pivot, slide, spintake;
    Servo claw;

    public void init() {
        pivot = myOp.hardwareMap.dcMotor.get("pivot");
        slide = myOp.hardwareMap.dcMotor.get("slide");
        claw = myOp.hardwareMap.servo.get("claw");
        spintake = myOp.hardwareMap.dcMotor.get("spintake");

        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spintake.setMode(DcMotor.RunMode.RUN_USING_ENCODER););
        claw.scaleRange(0, 1);

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivot.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        spintake.setDirection(DcMotor.Direction.FORWARD);
    }

    public void moveArm(int position, int angle) {
        
    }

}
