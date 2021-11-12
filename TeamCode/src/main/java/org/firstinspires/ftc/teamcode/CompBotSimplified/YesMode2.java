package org.firstinspires.ftc.teamcode.CompBotSimplified;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous
@Disabled
public class YesMode2 extends LinearOpMode {
    public final static double distanceK = 384.5/(100*Math.PI)*25.4, corrCoeff = 0.05, corrCoeff2 = 1;

    public DcMotor fl = null, fr = null, bl = null, br = null;
    public RevIMU imu = null;
    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = new RevIMU(hardwareMap,"imu");
        imu.init(parameters);

        fl = hardwareMap.get(DcMotor.class,"fl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        bl = hardwareMap.get(DcMotor.class,"bl");
        br = hardwareMap.get(DcMotor.class,"br");

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sleep(200);

        double dForward = 12, dStrafe = 12, sForward = 0.20, sStrafe = 0.20;

        fl.setTargetPosition(fl.getCurrentPosition() + (int) -(distanceK*(dForward+dStrafe)));
        fr.setTargetPosition(fr.getCurrentPosition() + (int) (distanceK*(dForward-dStrafe)));
        bl.setTargetPosition(bl.getCurrentPosition() + (int) -(distanceK*(dForward-dStrafe)));
        br.setTargetPosition(br.getCurrentPosition() + (int) (distanceK*(dForward+dStrafe)));

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("calc positions",Arrays.toString(new int[]{fl.getTargetPosition(), fr.getTargetPosition(), bl.getTargetPosition(), br.getTargetPosition()}));
        telemetry.addData("actual", Arrays.toString(new double[]{fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition()}));
        telemetry.update();

        waitForStart();


        double initialHeading = imu.getHeading(), error=0;
        while(fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy() && !isStopRequested()) {
            error = imu.getHeading() - initialHeading;

            if(fl.isBusy()) {
                fl.setPower(-(sForward + sStrafe) + corrCoeff*error);
            } else {
                fl.setPower(MathUtils.clamp((fl.getCurrentPosition()-fl.getTargetPosition() < 0 ? -1 : 1)*corrCoeff*error, -1, 1));
            }if(fr.isBusy()) {
                fr.setPower(sForward - sStrafe - corrCoeff*error);
            } else {
                fr.setPower(MathUtils.clamp((fr.getCurrentPosition()-fr.getTargetPosition() < 0 ? -1 : 1)*corrCoeff*error,-1,1));
            }if(bl.isBusy()) {
                bl.setPower(-(sForward - sStrafe) + corrCoeff*error);
            } else {
                bl.setPower(MathUtils.clamp((bl.getCurrentPosition()-bl.getTargetPosition() < 0 ? -1 : 1)*corrCoeff*error,-1,1));
            } if(br.isBusy()) {
                br.setPower(sForward + sStrafe - corrCoeff*error);
            } else {
                br.setPower(MathUtils.clamp((br.getCurrentPosition()-br.getTargetPosition() < 0 ? -1 : 1)*corrCoeff*error,-1,1));
            }
            telemetry.addData("calc positions",Arrays.toString(new int[]{fl.getTargetPosition(), fr.getTargetPosition(), bl.getTargetPosition(), br.getTargetPosition()}));
            telemetry.addData("error",error);
            double[] actualPositions = {fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition()};
            telemetry.addData("actual", Arrays.toString(actualPositions));
            telemetry.update();
        }

        while(!isStopRequested()) {
            error = imu.getHeading() - initialHeading;
            fl.setPower(MathUtils.clamp((fl.getCurrentPosition()-fl.getTargetPosition() < 0 ? -1 : 1)*corrCoeff2*error, -1, 1));
            fr.setPower(MathUtils.clamp((fr.getCurrentPosition()-fr.getTargetPosition() < 0 ? -1 : 1)*corrCoeff2*error,-1,1));
            bl.setPower(MathUtils.clamp((bl.getCurrentPosition()-bl.getTargetPosition() < 0 ? -1 : 1)*corrCoeff2*error,-1,1));
            br.setPower(MathUtils.clamp((br.getCurrentPosition()-br.getTargetPosition() < 0 ? -1 : 1)*corrCoeff2*error,-1,1));
            telemetry.addData("calc positions",Arrays.toString(new int[]{fl.getTargetPosition(), fr.getTargetPosition(), bl.getTargetPosition(), br.getTargetPosition()}));
            telemetry.addData("error",error);
            double[] actualPositions = {fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition()};
            telemetry.addData("actual", Arrays.toString(actualPositions));
            telemetry.update();
        }
    }
}
