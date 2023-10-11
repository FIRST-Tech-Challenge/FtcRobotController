package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//package org.firstinspires.ftc.teamcode;

//        import com.qualcomm.hardware.bosch.BNO055IMU;
//        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 //       import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 //       import com.qualcomm.robotcore.hardware.ColorSensor;
 //       import com.qualcomm.robotcore.hardware.DcMotor;
 //       import com.qualcomm.robotcore.hardware.DistanceSensor;
@Autonomous(name="noncontrolledpointscoringcode")
public class noncontrolledrobotpointscoringcode extends LinearOpMode {


    DcMotor motorLeft;
    DcMotor motorRight;
//        DcMotor frontLeft;
//        DcMotor frontRight;
//        ColorSensor color1;
//        DistanceSensor distance1;
//        BNO055IMU imu;

    @Override
    public void runOpMode() {
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        //       frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        //         frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        //           color1 = hardwareMap.get(ColorSensor.class, "color1");
//            distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
//            imu = hardwareMap.get(BNO055IMU.class, "imu");
    }
    }