package org.firstinspires.ftc.robotcontroller.internal;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class GyroCode {

    public BNO055IMU gyro;
    LinearOpMode opMode;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters;

   /* public GyroCode(LinearOpMode opMode) throws InterruptedException{

       // public double getCurrentGyro(){

      //  }

        public double getTrueDiff(double goalTurn){

        }
    }*/
}
