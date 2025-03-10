package org.firstinspires.ftc.teamcode.drive.teleop;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class Teleop extends OpMode {

   //INTAKE

    Servo rotate;
    Servo garra;
    Servo pleft;
    Servo pright;
    Servo lright;
    Servo lleft;

    //CAMERA
    public static Limelight3A limelight;
    public static double position(double angle) {
        return (angle + 15) / 30;
    }

    //OUTTAKE

    DcMotor poliaright;
    DcMotor polialeft;
    Servo Bright;
    Servo Bleft;
    Servo garrinha;
    double ticks = 2800.5;
    double newTarget;

    //MOVIMENTACAO

    DcMotor frontLeft, frontRight, backLeft, backRight;
    IMU imu;
    double headingOffset = 0.0;

    @Override
    public void init() {
        //INTAKE
        lright = hardwareMap.get(Servo.class,"lright");
        lleft = hardwareMap.get(Servo.class,"lleft");
        rotate = hardwareMap.get(Servo.class,"rotate");
        garra = hardwareMap.get(Servo.class,"garra");
        pleft = hardwareMap.get(Servo.class,"pleft");
        pright = hardwareMap.get(Servo.class,"pright");

        //CAMERA
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        //OUTTAKE
        poliaright = hardwareMap.get(DcMotor.class, "poliaright");
        polialeft = hardwareMap.get(DcMotor.class, "polialeft");
        Bright = hardwareMap.get(Servo.class,"bright");
        Bleft = hardwareMap.get(Servo.class,"bleft");
        garrinha = hardwareMap.get(Servo.class,"garrinha");

        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        poliaright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        polialeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //MOVIMENTACAO
        frontLeft = hardwareMap.get(DcMotor.class, "odor");
        frontRight = hardwareMap.get(DcMotor.class, "odom");
        backLeft = hardwareMap.get(DcMotor.class, "odol");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        telemetry.addData("Hardware: ", "Initialized");
    }

    @Override
    public void loop() {
        //INTAKE
        if (gamepad2.dpad_down){
            garra.setPosition(0.7);
            pleft.setPosition(0.8);
            pright.setPosition(0.2);
            lright.setPosition(1);
            lleft.setPosition(0);

        }
        if (gamepad1.dpad_up) {
            lright.setPosition(0.6);
            lleft.setPosition(0.7);
            sleep(200);
            garra.setPosition(0.3);
            pleft.setPosition(0);
            pright.setPosition(1);
        }

        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                rotate.setPosition(position(limelight.getLatestResult().getTyNC()));
                telemetry.addData("rotate", rotate.getPosition());
            } else {
                telemetry.addData("Limelight", "No data available");
            }
            telemetry.update();
        }

        //OUTTAKE

        if (gamepad1.dpad_up){
            viperslide1Up(-1);
            viperslide2Up(1);
        }
        if (gamepad1.dpad_left){
            viperslide1Up(-3);
            viperslide2Up(3);
        }
        if (gamepad1.dpad_right){
            viperslide1Down();
            viperslide2Down();
        }
        int currentPosition1 = poliaright.getCurrentPosition();
        int currentPosition2 = polialeft.getCurrentPosition();
        int MIN_POSITION = 0;

        if (currentPosition1 <= MIN_POSITION) {
            poliaright.setPower(0);
        }
        if (currentPosition2 <= MIN_POSITION){
            polialeft.setPower(0);
        }
        if (gamepad1.a) {
            Bright.setPosition(1);
            Bleft.setPosition(0);
        }
        if (gamepad1.b){
            Bright.setPosition(0.2);
            Bleft.setPosition(0.8);
        }
        if (gamepad1.x){
            garrinha.setPosition(0.4);
        }
        if (gamepad1.y){
            garrinha.setPosition(0);
        }
        //MOVIMENTACAO

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double currentHeading = -angles.getYaw(AngleUnit.RADIANS) - headingOffset;

        double y = -gamepad1.left_stick_y; // Forward/Backward
        double x = -gamepad1.left_stick_x;  // Strafe Left/Right
        double turn = gamepad1.right_stick_x; // Rotation

        double cosA = Math.cos(currentHeading);
        double sinA = Math.sin(currentHeading);
        double fieldX = x * cosA - y * sinA;
        double fieldY = x * sinA + y * cosA;

        double frontLeftPower = fieldY + fieldX + turn;
        double frontRightPower = fieldY - fieldX - turn;
        double backLeftPower = fieldY - fieldX + turn;
        double backRightPower = fieldY + fieldX - turn;

        double max = Math.max(1.0, Math.abs(frontLeftPower));
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        frontLeft.setPower(frontLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        backLeft.setPower(backLeftPower / max);
        backRight.setPower(backRightPower / max);
    }
    public void viperslide1Up(int turnage) {
        newTarget = ticks / turnage;
        poliaright.setTargetPosition((int) newTarget);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide2Up(int turnage) {
        newTarget = ticks / turnage;
        polialeft.setTargetPosition((int) newTarget);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide1Down() {
        poliaright.setTargetPosition(0);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide2Down() {
        polialeft.setTargetPosition(0);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}