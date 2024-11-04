package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.localization.OdometryFuse;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtilities;

@TeleOp(name = "Fresh OdometryFuse")
public class TestOdometryFuse  extends LinearOpMode {
    SparkFunOTOS myOtos;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor rightBack;
    @Override
    public void runOpMode() throws InterruptedException {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS");
        leftFront = hardwareMap.get(DcMotor.class, "fLeft");
        rightFront = hardwareMap.get(DcMotor.class, "fRight");
        rightBack = hardwareMap.get(DcMotor.class, "bRight");
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap ,this, telemetry);
        OdometryFuse odometryFuse = new OdometryFuse(myOtos, rightFront, leftFront, rightBack);
        telemetry.addData("" + odometryFuse.configureOtos(myOtos), "");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("" + odometryFuse.SparkUpdateData(), "");
            telemetry.addLine();
            telemetry.addData("" + odometryFuse.WheelUpdateData(), "");
            telemetry.addLine();
            telemetry.addData("" + odometryFuse.averageUpdateData(), "");
            telemetry.update();
            Log.d("alan", "updated position on odometry");
        }
    }
}
