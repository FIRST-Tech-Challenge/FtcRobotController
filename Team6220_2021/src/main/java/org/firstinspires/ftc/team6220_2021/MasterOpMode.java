package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.DriverInput;

public abstract class MasterOpMode extends LinearOpMode {
    // Motors
    public static DcMotor motorFL;
    public static DcMotor motorFR;
    public static DcMotor motorBL;
    public static DcMotor motorBR;
    public static DcMotor motorBelt;
    public static DcMotor motorArm;
    public static DcMotor motorLeftDuck;
    public static DcMotor motorRightDuck;

    // Other Devices
    public static Servo servoArm;
    public static Servo servoGrabber;

    // Create Drivers
    public DriverInput driver1;
    public DriverInput driver2;

    // IMUs
    public BNO055IMU imu;

    // Initializes the motors, servos, IMUs, and drivers
    public void Initialize() {
        // Drive train motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorBelt = hardwareMap.dcMotor.get("motorBelt");
        motorArm = hardwareMap.dcMotor.get("motorArm");
        motorLeftDuck = hardwareMap.dcMotor.get("motorLeftDuck");
        motorRightDuck = hardwareMap.dcMotor.get("motorRightDuck");

        servoArm = hardwareMap.servo.get("servoArm");
        servoGrabber = hardwareMap.servo.get("servoGrabber");

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeftDuck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightDuck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setTargetPosition(0);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        // Retrieve and initialize the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void forward(double driveInches, double power) {
        Initialize();

        double TotalTicks = 537.6 * driveInches / 12.57;
        int targetticks = (int) TotalTicks;
        motorFL.setTargetPosition(targetticks);
        motorBL.setTargetPosition(targetticks);
        motorFR.setTargetPosition(-targetticks);
        motorBR.setTargetPosition(-targetticks);
        motorFL.setPower(power);
        motorBL.setPower(power);
        motorFR.setPower(power);
        motorBR.setPower(power);
        while (motorBR.isBusy() || motorBL.isBusy() || motorFL.isBusy() || motorFR.isBusy()) {
            telemetry.addData("Currently Running", motorFL.getCurrentPosition());
        }
        pauseMillis(100);
    }

    public void stopBase() {
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }

    public void blueDuck() {
        motorLeftDuck = hardwareMap.dcMotor.get("motorDuck");
        double x = -0.7;
        while (true) {
            motorLeftDuck.setPower(x);
            pauseMillis(150);
            x -= 0.05;
            telemetry.addData("duckPower", motorLeftDuck.getPower());
            telemetry.update();
            if (x <= -0.85) {
                pauseMillis(1500);
                motorLeftDuck.setPower(-.1);
                pauseMillis(30);
                motorLeftDuck.setPower(0);
                x = 0.7;
                break;
            }
        }
    }

    public void turnAngle(double turnDegree) {
        Initialize();

        double TotalTicks = 537.6 * 20 / 4 * (turnDegree / 360);
        int targetticks = (int) TotalTicks;
        motorFL.setTargetPosition(targetticks);
        motorBL.setTargetPosition(targetticks);
        motorFR.setTargetPosition(targetticks);
        motorBR.setTargetPosition(targetticks);
        motorFL.setPower(0.9);
        motorBL.setPower(0.9);
        motorFR.setPower(0.9);
        motorBR.setPower(0.9);
        while (motorBR.isBusy() || motorBL.isBusy() || motorFL.isBusy() || motorFR.isBusy()) {
            telemetry.addData("Currently Running", motorFL.getCurrentPosition());
        }
        pauseMillis(100);

    }

    // Pauses for time milliseconds
    public void pauseMillis(double time) {
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time && opModeIsActive()) {
            idle();
        }
    }
}