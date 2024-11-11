package com.kalipsorobotics.test; import com.kalipsorobotics.localization.OdometryFuse;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "OdometryFuse testing")
public class TestOdometryFuse  extends LinearOpMode {
    SparkFunOTOS myOtos;
    DcMotor rightFront;
    DcMotor rightBack;
    @Override
    public void runOpMode() throws InterruptedException {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sprk sensor OTOS");
        rightFront = hardwareMap.get(DcMotor.class, "fRight");
        rightBack = hardwareMap.get(DcMotor.class, "bRight");
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap ,this, telemetry);
        OdometryFuse odometryFuse = new OdometryFuse(myOtos, rightFront, rightBack);
        telemetry.addData("" + odometryFuse.configureOtos(myOtos), "");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("x: " + odometryFuse.CollectData().getX(), "");
            telemetry.addData("y: " + odometryFuse.CollectData().getY(), "");
            telemetry.addData("H: " + odometryFuse.HeadingUpdateData(), "");
            telemetry.update();
        }
    }
}