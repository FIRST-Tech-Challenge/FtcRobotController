package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@Autonomous(name = "RedLeft", group = "MecanumBot")
public class RedLeft extends LinearOpMode {

    DcMotor m1, m2, m3, m4;
    public void runOpMode(){
        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
//        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
//        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
//        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);

        //ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        waitForStart();

        String color = "red";
        if (color == "red") {
            forwardDrive("forward", 950, 1);
            StrafeDrive("right", 1260, 1);
            StrafeDrive("left", 2100,1);

        } else if (color == "red") {
            forwardDrive("forward", 1000, 1);
            StrafeDrive("left", 1260, 1);
            StrafeDrive("right", 1200,1);
        }
        else {
            forwardDrive("forward", 1000, 1);
            StrafeDrive("left", 1260, 1);
            StrafeDrive("right", 450, 1);
        }



//        telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
//        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//        telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
//
//        telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
//        telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
//        telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
//        telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
//        telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
//                m3.getCurrentPosition(), m4.getCurrentPosition());
//        telemetry.update();

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

    }

    public void StrafeDrive(String direction,
                            int time, double speed){
        if (direction.equals("left")){
            m1.setPower(speed);
            m2.setPower(-speed);
            m3.setPower(speed);
            m4.setPower(-speed);
            sleep(time);
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
        }
        if (direction.equals("right")){
            m1.setPower(-speed);
            m2.setPower(speed);
            m3.setPower(-speed);
            m4.setPower(speed);
            sleep(time);
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
        }
        sleep(1000);
    }

    public void forwardDrive(String direction,
                             long time, double speed){
        if (direction.equals("forward")){
            m1.setPower(speed);
            m2.setPower(speed);
            m3.setPower(speed);
            m4.setPower(speed);
            sleep(time);
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);


        }
        if (direction.equals("back")){
            m1.setPower(-1);
            m2.setPower(-1);
            m3.setPower(-1);
            m4.setPower(-1);
            sleep(1000);
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
        }
        sleep(1000);
    }

    public void rotation(String direction,
                         long time, double speed){
        if (direction.equals("clockwise")){
            m1.setPower(speed);
            m2.setPower(speed);
            m3.setPower(-speed);
            m4.setPower(-speed);
            sleep(time);
        }

        if (direction.equals("counter-clockwise")){
            m1.setPower(-speed);
            m2.setPower(-speed);
            m3.setPower(speed);
            m4.setPower(speed);
            sleep(time);
        }
    }



}
