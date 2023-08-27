package org.firstinspires.ftc.teamcode.Ethan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class EthanAuto extends LinearOpMode {

    private static final double P_VALUE = 0.006;

    //301 = circumferance mm
    //537.7, ticks per motor revolution
    //1.4, gear ratio
    //converting mm to ticks
    private static final double MM_TO_TICKS = (537.7/1.4)/301.59;

    public void AutoForward(double targetDistanceInMM) throws InterruptedException {

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



        waitForStart();

        double targetPos = targetDistanceInMM * MM_TO_TICKS + lFront.getCurrentPosition();

        /*lFront.setPower(0.1);
        lBack.setPower(0.1);
        rFront.setPower(0.1);
        rBack.setPower(0.1);

        while (lFront.getCurrentPosition() < targetPos && opModeIsActive()) {}

        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);*/

        while (lFront.getCurrentPosition() <= Math.abs(targetPos) && opModeIsActive()) {

            double error = targetPos - lFront.getCurrentPosition();

            lFront.setPower(P_VALUE*error);
            lBack.setPower(P_VALUE*error);
            rFront.setPower(P_VALUE*error);
            rBack.setPower(P_VALUE*error);

            telemetry.addLine("moving");
            telemetry.update();
        }



        telemetry.addLine(String.valueOf(lFront.getCurrentPosition()/MM_TO_TICKS));
        telemetry.update();

        double oldTick = 0;

        while (true) {
            double newTick = lFront.getCurrentPosition();


            if (newTick == oldTick) {
                break;
            }

            oldTick = newTick;

            Thread.sleep(5);
        }

        telemetry.addLine("done");
        telemetry.update();

    }

    public void AutoTurning (double targetAngle) throws InterruptedException  {

        DcMotor lFront = hardwareMap.dcMotor.get("Left front");
        DcMotor lBack = hardwareMap.dcMotor.get("Left back");
        DcMotor rFront = hardwareMap.dcMotor.get("Right front");
        DcMotor rBack = hardwareMap.dcMotor.get("Right back");

        lFront.setDirection(DcMotorSimple.Direction.REVERSE);
        lBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rBack.setDirection(DcMotorSimple.Direction.FORWARD);


        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        double angleInTicks = targetAngle*(850./90);
        double P_VALUE_FOR_TURNING = 0.001;


        while (opModeIsActive()) {
           double angleError = angleInTicks-lFront.getCurrentPosition();

           telemetry.addData("left front ticks", lFront.getCurrentPosition());
           telemetry.update();

           lFront.setPower(P_VALUE_FOR_TURNING*angleError);
           lBack.setPower(P_VALUE_FOR_TURNING*angleError);
           rFront.setPower(-P_VALUE_FOR_TURNING*angleError);
           rBack.setPower(-P_VALUE_FOR_TURNING*angleError);
       }

        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        AutoTurning(90);

        /*AutoForward(300);
        AutoForward(-300);*/

    }
}
