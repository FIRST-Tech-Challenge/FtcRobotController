package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled

public class DTMotorsTest extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, arm, slides;
    private static final double degreesPerTick = 0.1602564102564103;
    private static final double inchesPerTick = 0.0287253141831239;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "bl");
        leftBack = hardwareMap.get(DcMotorEx.class, "fl");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        rightBack = hardwareMap.get(DcMotorEx.class, "br");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        slides = hardwareMap.get(DcMotorEx.class, "slides");




        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while(opModeIsActive()){
            //left front: slides, left back: arm
            double lfencoder = leftFront.getCurrentPosition();
            double lbencoder = leftBack.getCurrentPosition();
            double rfencoder = rightFront.getCurrentPosition();
            double rbencoder = rightBack.getCurrentPosition();
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double r = -gamepad1.right_stick_x;
            double normalize = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
            double flPower = (y+x-r);
            double blPower = (y-x-r);
            double brPower = (y+x+r);
            double frPower = (y-x+r);
            leftFront.setPower((flPower/normalize)); // works
            leftBack.setPower((blPower/normalize)); //works
            rightBack.setPower((brPower/normalize)); //works
            rightFront.setPower((frPower/normalize));

            telemetry.addData("slides: ", lfencoder*inchesPerTick);
            telemetry.addData("arm: ", lbencoder*degreesPerTick);
            telemetry.update();

        }
    }
}
