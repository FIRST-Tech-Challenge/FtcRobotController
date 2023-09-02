package org.firstinspires.ftc.teamcode.Ethan;

import android.graphics.Path;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class EthanRobot {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    LinearOpMode opMode;

    DcMotor lFront;
    DcMotor rFront;
    DcMotor lBack;
    DcMotor rBack;
    public EthanRobot(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        this.telemetry = telemetry;
    }


    public void setUpMotors() {
        lFront = hardwareMap.dcMotor.get("Left front");
        rFront = hardwareMap.dcMotor.get("Right front");
        lBack = hardwareMap.dcMotor.get("Left back");
        rBack = hardwareMap.dcMotor.get("Right back");

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
    }


    public void AForward(double targetDistanceInMM) throws InterruptedException {





        final double P_VALUE = 0.006;

        //301 = circumferance mm
        //537.7, ticks per motor revolution
        //1.4, gear ratio
        //converting mm to ticks
        final double MM_TO_TICKS = (537.7/1.4)/301.59;


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

        double error = targetPos - lFront.getCurrentPosition();

        long lastCheckMillis = System.currentTimeMillis();
        long millis = System.currentTimeMillis();

        double oldTick = lFront.getCurrentPosition();

        boolean isStopped = false;

        while (!(error <= 10 && isStopped) && opMode.opModeIsActive()) {

            millis = System.currentTimeMillis();

            telemetry.addLine(String.valueOf(targetPos));
            telemetry.addLine(String.valueOf(lFront.getCurrentPosition()));

            error = targetPos - lFront.getCurrentPosition();
            telemetry.addLine(String.valueOf(error));

            lFront.setPower(P_VALUE*error);
            lBack.setPower(P_VALUE*error);
            rFront.setPower(P_VALUE*error);
            rBack.setPower(P_VALUE*error);

            telemetry.addLine("moving");



            if (millis > lastCheckMillis+500) {
                lastCheckMillis = millis;
                double newTicks = lFront.getCurrentPosition();

                if (oldTick == newTicks) {
                    isStopped = true;
                }

                oldTick = newTicks;


            }



            telemetry.update();
        }

        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);

        telemetry.addLine(String.valueOf(lFront.getCurrentPosition()/MM_TO_TICKS));

        /*double oldTick = 0;

        while (true) {
            double newTick = lFront.getCurrentPosition();


            if (newTick == oldTick) {
                break;
            }

            oldTick = newTick;

            Thread.sleep(5);
        }*/

        telemetry.addLine("done");
        telemetry.update();



    }
}
