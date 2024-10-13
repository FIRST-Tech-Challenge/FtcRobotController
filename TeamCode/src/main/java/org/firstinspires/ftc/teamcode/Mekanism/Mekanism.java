package org.firstinspires.ftc.teamcode.Mekanism;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Mekanism {
    LinearOpMode myOp;

    DcMotor pivot, slide, spintake;
    Servo claw;


    int COUNTS_PER_INCH = 1120;
    int COUNTS_PER_DEGREE = 1120;

    public void init() {
        pivot = myOp.hardwareMap.dcMotor.get("pivot");
        slide = myOp.hardwareMap.dcMotor.get("slide");
        claw = myOp.hardwareMap.servo.get("claw");
        spintake = myOp.hardwareMap.dcMotor.get("spintake");

        pivot.setTargetPosition(0);
        slide.setTargetPosition(0);

        pivot.setPower(1);
        slide.setPower(1);

        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spintake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw.scaleRange(0, 1);

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivot.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        spintake.setDirection(DcMotor.Direction.FORWARD);
    }

    // set the pos and angle for arm independently
    public void basicMoveArm(int position, int angle) {

        slide.setTargetPosition(position);
        pivot.setTargetPosition(angle);

    }


    // In IN.
    // uses x and y to calculate arm angle and length
    public void moveXY(int x, int y) {

        if (x < 0) {
            x = 0;
        }
        if (y < 0) {
            y = 0;
        }

        int armLen = (int) Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        int armAngle = (int) Math.atan((double) y / (double) x);

        slide.setTargetPosition(armLen * COUNTS_PER_INCH);

        pivot.setTargetPosition(armAngle * COUNTS_PER_DEGREE);
    }


    // sets the pos for claw
    public void moveClaw(double position) {
        claw.setPosition(position);
    }
}
