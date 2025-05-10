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
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp
public class Teleop extends OpMode {

    //INTAKE

    Servo rotate;
    Servo garra;
    Servo pleft;
    Servo pright;
    Servo lright;
    Servo lleft;
    //OUTTAKE

    DcMotor poliaright;
    DcMotor polialeft;
    Servo Bright;
    Servo Bleft;
    Servo garrinha;
    double ticks = 2800.5;
    double ticks2 = 4500;
    double ticks3 = 1000;
    double newTarget;

    //MOVIMENTACAO

    DcMotor frontLeft, frontRight, backLeft, backRight;
    IMU imu;

    @Override
    public void init() {
        //INTAKE
        lright = hardwareMap.get(Servo.class, "lright");
        lleft = hardwareMap.get(Servo.class, "lleft");
        rotate = hardwareMap.get(Servo.class, "rotate");
        garra = hardwareMap.get(Servo.class, "garra");
        pleft = hardwareMap.get(Servo.class, "pleft");
        pright = hardwareMap.get(Servo.class, "pright");

        //OUTTAKE
        poliaright = hardwareMap.get(DcMotor.class, "poliaright");
        polialeft = hardwareMap.get(DcMotor.class, "polialeft");
        Bright = hardwareMap.get(Servo.class, "bright");
        Bleft = hardwareMap.get(Servo.class, "bleft");
        garrinha = hardwareMap.get(Servo.class, "garrinha");

        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        poliaright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        polialeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //MOVIMENTACAO
        frontLeft = hardwareMap.get(DcMotor.class, "odol");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "odor");
        backRight = hardwareMap.get(DcMotor.class, "odom");

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Hardware: ", "Initialized");
    }
        public void loop(){
            //INTAKE
            if (gamepad2.dpad_down) {
                garra.setPosition(0.7);
                sleep(200);
                rotate.setPosition(0.7);
                pleft.setPosition(0.8);
                pright.setPosition(0.2);
                lright.setPosition(1);
                lleft.setPosition(0.1);
                sleep(500);
                garra.setPosition(0.3);
            }
            if (gamepad2.a) {
                garra.setPosition(0.3);
            }
            if (gamepad2.left_bumper){
                rotate.setPosition(1);
            }
            if (gamepad2.right_bumper){
                rotate.setPosition(0.5);
            }
            if (gamepad2.dpad_up) {
                lright.setPosition(0.6);
                lleft.setPosition(0.7);
                sleep(200);
                garra.setPosition(0.3);
                pleft.setPosition(0);
                pright.setPosition(1);
            }

            //OUTTAKE

            if (gamepad1.dpad_up) {
                viperslide1Up(-1);
                viperslide2Up(1);
            }
            if (gamepad1.dpad_left) {
                viperslide1Clipar(-3);
                viperslide2Clipar(3);
            }
            if (gamepad1.dpad_right){
                viperslide1Clip(-2);
                viperslide2Clip(2);
            }
            if (gamepad1.dpad_down) {
                viperslide1Down();
                viperslide2Down();
                Bright.setPosition(1);
                Bleft.setPosition(0);
                sleep(1100);
                polialeft.setPower(0);
                poliaright.setPower(0);
            }
            if (gamepad1.y) {
                Bright.setPosition(1);
                Bleft.setPosition(0);
            }
            if (gamepad1.x) {
                Bright.setPosition(0.4);
                Bleft.setPosition(0.6);
            }
            if (gamepad1.b) {
                garrinha.setPosition(0.4);
            }
            if (gamepad1.a) {
                garrinha.setPosition(0);
            }
            if (gamepad1.right_bumper){
                climb();
            }
            //MOVIMENTACAO
            anda();
        }
        public void viperslide1Up ( int turnage){
        newTarget = ticks / turnage;
        poliaright.setTargetPosition((int) newTarget);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide2Up ( int turnage){
        newTarget = ticks / turnage;
        polialeft.setTargetPosition((int) newTarget);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide1Down () {
        poliaright.setTargetPosition(0);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide2Down () {
        polialeft.setTargetPosition(0);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide1Clip ( int turnage){
        newTarget = ticks2 / turnage;
        poliaright.setTargetPosition((int) newTarget);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide2Clip ( int turnage){
        newTarget = ticks2 / turnage;
        polialeft.setTargetPosition((int) newTarget);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide1Clipar ( int turnage){
        newTarget = ticks3 / turnage;
        poliaright.setTargetPosition((int) newTarget);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide2Clipar ( int turnage){
        newTarget = ticks3 / turnage;
        polialeft.setTargetPosition((int) newTarget);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void climb(){
        viperslide1Down();
        viperslide2Down();
    }
    private void anda(){
        double forward = gamepad2.left_stick_y;
        double strafe = -gamepad2.left_stick_x;
        double turn = -gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 0.5);

        frontRight.setPower((forward - strafe - turn) / denominator);
        frontLeft.setPower((forward + strafe + turn) / denominator);
        backLeft.setPower((forward - strafe + turn) / denominator);
        backRight.setPower((forward + strafe - turn) / denominator);
    }
}