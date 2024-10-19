package org.firstinspires.ftc.teamcode.CompBot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MekanismConfig {

    public LinearOpMode myOp;
    public MekanismConfig(LinearOpMode opMode) {
        myOp = opMode;
    }

    DcMotor pivot, slide, spintake;
    Servo claw, wrist;


    int COUNTS_PER_INCH = 1120;
    int COUNTS_PER_DEGREE = 1120;


    /**
     * Initializes the mechanism of the robot.<br>
     * Starts all the devices and maps where they go
     * As well as sets direction and whether motors run with encoders or not
     */
    public void initMekanism() {

        // Init slaw, claw, and pivot
        pivot = myOp.hardwareMap.get(DcMotor.class, "pivot");
        slide = myOp.hardwareMap.get(DcMotor.class, "slide");


        pivot.setTargetPosition(0);
        slide.setTargetPosition(0);

        pivot.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);

        pivot.setMode(STOP_AND_RESET_ENCODER);
        slide.setMode(STOP_AND_RESET_ENCODER);

        pivot.setMode(RUN_USING_ENCODER);
        slide.setMode(RUN_USING_ENCODER);

        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sets maximum allowed power to 1
        pivot.setPower(1);
        slide.setPower(1);



        wrist = myOp.hardwareMap.get(Servo.class, "wrist");
        claw = myOp.hardwareMap.get(Servo.class, "claw");
        claw.scaleRange(0, 1);


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
