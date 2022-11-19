package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Variables.*;

import android.graphics.drawable.GradientDrawable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="LukasAuto", group = "A")
//@Disabled
public class LukasAutonomous extends DriveMethods {



    @Override
    public void runOpMode() {
        initMotorsBlue();
        calibrateNavXIMU();

        waitForStart();

        driveForDistance(1.95,Direction.LEFT,0.4,0);
        sleep(100);
        driveForDistance(0.1,Direction.FORWARD,0.4,0);
        sleep(500);
        driveForDistance(0.1,Direction.BACKWARD,0.4,0);
        sleep(100);
        driveForDistance(0.3,Direction.RIGHT,0.4,0);
        sleep(100);
        driveForDistance(1.4,Direction.FORWARD,0.4,0);
        sleep(100);
        driveForDistance(1.4,Direction.BACKWARD,0.4,0);
        sleep(100);
        driveForDistance(0.3,Direction.LEFT,0.4,0);
        sleep(100);
        driveForDistance(0.1,Direction.FORWARD,0.4,0);
        sleep(500);
        driveForDistance(0.1,Direction.BACKWARD,0.4,0);






        while (opModeIsActive()) {
            driveForDistance(0.3,Direction.RIGHT,0.4,0);
            sleep(100);
            driveForDistance(1.4,Direction.FORWARD,0.4,0);
            sleep(100);
            driveForDistance(1.4,Direction.BACKWARD,0.4,0);
            sleep(100);
            driveForDistance(0.28,Direction.LEFT,0.4,0);
            sleep(100);
            driveForDistance(0.1,Direction.FORWARD,0.4,0);
            sleep(500);
            driveForDistance(0.1,Direction.BACKWARD,0.4,0);


        }
    }









}

