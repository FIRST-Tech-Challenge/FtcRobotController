package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;  //This is the package for controlling the IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Math;  //This is the standard Java package for a variety of math functions

@TeleOp

public class StrippedSteveBasicOPMode extends LinearOpMode{
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    //Declaration for the servo for the puller
    private Servo back_foundation_puller;
    //Declaration for the four drive motors
    private DcMotor back_left_wheel;
    private DcMotor back_right_wheel;
    private DcMotor front_left_wheel;
    private DcMotor front_right_wheel;
    //Declaration for the sensors.  These classes were created to work with the default/supported
    //sensors on the robot.  Technically, if we ever run into a sensor that has the same interface
    //as one of the ports on the Expansion Hub, you could design your own interface.
    private DistanceSensor color_sensor;
    private DistanceSensor distance_sensor;
    private ColorSensor front_color_sensor;
    //Declaration for the Inertial Measurement Unit (IMU)--note:  This is the built in gyroscope
    //in the Expansion Hub.
    private BNO055IMU imu;
    private DcMotor lift_motor;
    private DcMotor clamp_motor;
    //Below is the declaration for the digital "red/blue" switch we have on our old robot.
    //The system has a collection of other interfaces that can be used regularly.
    //NOTE:  For those of you using motion control/needing start/stop locations, you could use
    //something like this for a limit switch or something--and then use it as part of a closed loop
    //control algorithm.
    private DigitalChannel switch_;


    private class Chassis {
        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;


        private void SetMotors (double drive, double strafe, double rotate) {
            this.frontLeft = -drive + strafe + rotate;
            this.backLeft = -drive - strafe + rotate;
            this.frontRight = drive + strafe + rotate;
            this.backRight = drive - strafe + rotate;
        }


        private void Drive () {
            front_right_wheel.setPower(this.frontRight);
            front_left_wheel.setPower(this.frontLeft);
            back_left_wheel.setPower(this.backLeft);
            back_right_wheel.setPower(this.backRight);
        }
    }



    enum OperState {
        NORMALDRIVE,
        Example1
    }

    @Override
    public void runOpMode() {

        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        back_foundation_puller = hardwareMap.get(Servo.class, "back foundation puller");
        back_left_wheel = hardwareMap.get(DcMotor.class, "back left wheel");
        back_right_wheel = hardwareMap.get(DcMotor.class, "back right wheel");
        clamp_motor = hardwareMap.get(DcMotor.class, "clamp motor");
        color_sensor = hardwareMap.get(DistanceSensor.class, "color sensor");
        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance sensor");
        front_color_sensor = hardwareMap.get(ColorSensor.class, "front color sensor");
        front_left_wheel = hardwareMap.get(DcMotor.class, "front left wheel");
        front_right_wheel = hardwareMap.get(DcMotor.class, "front right wheel");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        lift_motor = hardwareMap.get(DcMotor.class, "lift motor");
        switch_ = hardwareMap.get(DigitalChannel.class, "switch ");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();  //in wrong spot--where is better?
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu.initialize(parameters);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        /*
        END OF INITIALIZATION SECTION
         */
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double drive;  //Power for forward and back motion
        double strafe; //Power for left and right motion
        double rotate; //Power for rotating the robot
        StrippedSteveBasicOPMode.Chassis superRobot = new StrippedSteveBasicOPMode.Chassis(); //instantiation of chassis class
        double rotationAngle = 0;  //passed as state argument for the set position in driveOpState
        StrippedSteveBasicOPMode.OperState driveOpState = StrippedSteveBasicOPMode.OperState.NORMALDRIVE; //instantiation of the state variable


        while (opModeIsActive()) {

            double zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
            double yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).secondAngle;
            double xAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).thirdAngle;


            switch (driveOpState) {
                case NORMALDRIVE :
                    drive = -this.gamepad1.left_stick_y;
                    strafe = -this.gamepad1.left_stick_x;
                    rotate = -this.gamepad1.right_stick_x;
                    superRobot.SetMotors (drive, strafe, rotate);
                    superRobot.Drive();

                    if (this.gamepad1.right_bumper) {
                        driveOpState = OperState.Example1;
                    }

                    break;

                case Example1:
                    driveOpState = OperState.NORMALDRIVE;
                    break;

                default :
                    break;
            }
        }
    }
}

