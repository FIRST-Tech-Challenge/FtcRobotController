package org.firstinspires.ftc.teamcode.Ethan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class EncoderMethod extends LinearOpMode {

    public void encoderMove(double targetDistance) throws InterruptedException {

        DcMotor lFront = hardwareMap.dcMotor.get("Left front");
        DcMotor rFront = hardwareMap.dcMotor.get("Right front");
        DcMotor lBack = hardwareMap.dcMotor.get("Left back");
        DcMotor rBack = hardwareMap.dcMotor.get("Right back");

        rBack.setDirection(DcMotorSimple.Direction.FORWARD);
        lBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rFront.setDirection(DcMotorSimple.Direction.FORWARD);
        lFront.setDirection(DcMotorSimple.Direction.REVERSE);


        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double pValue = 0.001;

        //301 = circumferance mm
        //537.7, ticks per motor revolution
        //1.4, gear ratio
        //converting mm to ticks
        double MM_TO_TICKS = (537.7/1.4)/301.59;

        waitForStart();

        double targetPos = targetDistance * MM_TO_TICKS + lFront.getCurrentPosition();

        /*lFront.setPower(0.1);
        lBack.setPower(0.1);
        rFront.setPower(0.1);
        rBack.setPower(0.1);

        while (lFront.getCurrentPosition() < targetPos && opModeIsActive()) {}

        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);*/

        while (opModeIsActive()) {

            double error = targetPos - lFront.getCurrentPosition();

            lFront.setPower(pValue*error);
            lBack.setPower(pValue*error);
            rFront.setPower(pValue*error);
            rBack.setPower(pValue*error);

        }



        telemetry.addLine(String.valueOf(lFront.getCurrentPosition()/MM_TO_TICKS));
        telemetry.update();

        while (opModeIsActive()) {
            double newTick = lFront.getCurrentPosition();
            double oldTick = 0;

            if (newTick == oldTick) {
                break;
            }

            oldTick = newTick;

            Thread.sleep(5);
        }

    }



    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

           encoderMove(600);

    }
}
