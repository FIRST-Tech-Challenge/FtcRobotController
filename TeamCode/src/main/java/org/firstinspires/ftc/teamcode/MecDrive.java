//  _____                _           _       _  _    __  _____ _  _
// |  ___| __ ___   __ _| |__   ___ | |_ ___| || |  / /_|___ /| || |
// | |_ | '__/ _ \ / _` | '_ \ / _ \| __/ __| || |_| '_ \ |_ \| || |_
// |  _|| | | (_) | (_| | |_) | (_) | |_\__ \__   _| (_) |__) |__   _|
// |_|  |_|  \___/ \__, |_.__/ \___/ \__|___/  |_|  \___/____/   |_|
//                 |___/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.speech.RecognitionService;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.drivebase.CenterStageDriveBase;


@TeleOp
public class MecDrive extends LinearOpMode {

    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx RL;
    private DcMotorEx RR;

    private DcMotorEx rightHook;
    private DcMotorEx leftHook;
    private DcMotorEx intake;
    private DcMotorEx Lift;
    private DcMotorEx RHang;
    private DcMotorEx LHang;
    private Servo RHook;
    private Servo LHook;
    private Servo intakeLift;
    private Servo airplane;
    private Servo Door;
    private Servo RLock;
    private Servo LLock;
    private Servo Pivot;
    private Servo SLift;

    private float Rservopos;
    private float Lservopos;

    private double leftStickX;
    private double leftStickY;
    private double rightStickX;
    private int LiftCounts;
    private float speedFactor;

    NormalizedColorSensor colorSensor;

    CenterStageDriveBase centerStageDriveBase;
    TrackingWheelIntegrator trackingWheelIntegrator;

    float[] hsvValues = new float[3];

    String pixcol;

    public void runOpMode() {



        trackingWheelIntegrator = new TrackingWheelIntegrator();

        FL= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "FL");
        FR= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "FR");
        RL= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "RL");
        RR= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "RR");

        intake= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "intake");
        Lift = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "Lift");
        LHang = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "LHang");
        RHang = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "RHang");


        intakeLift=(Servo) hardwareMap.get(Servo.class, "intakeLift");
        airplane=(Servo)  hardwareMap.get(Servo.class, "airplane");
        Door=(Servo)  hardwareMap.get(Servo.class, "Door");
        RLock=(Servo)  hardwareMap.get(Servo.class, "RLock");
        LLock=(Servo)  hardwareMap.get(Servo.class, "LLock");
        Pivot=(Servo)  hardwareMap.get(Servo.class, "Pivot");
        SLift=(Servo)  hardwareMap.get(Servo.class, "SLift");
        RHook=(Servo)  hardwareMap.get(Servo.class, "RHook");
        LHook=(Servo)  hardwareMap.get(Servo.class, "LHook");


        colorSensor= (NormalizedColorSensor) hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        centerStageDriveBase = new CenterStageDriveBase();
        centerStageDriveBase.init(hardwareMap);
        centerStageDriveBase.enablePID();
        Globals.robot=centerStageDriveBase;
        Globals.driveBase=centerStageDriveBase;
        Globals.trackingWheelIntegrator = trackingWheelIntegrator;
        Globals.opMode = this;
        Globals.robot.enableBrake(true);
        Rservopos = 1;
        Lservopos = -1;
        speedFactor = 1;

        Lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Lift.setTargetPosition(1);
        Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        telemetry.setMsTransmissionInterval(20);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            runGamepad();
        }




    }

    void runGamepad() {

        leftStickX = gamepad1.left_stick_x;
        leftStickY = gamepad1.left_stick_y;
        rightStickX = gamepad1.right_stick_x;


        if (gamepad1.a) {
            intakeLift.setPosition(.5);
            intake.setPower(1);
        }

        if (gamepad1.y) {
            intakeLift.setPosition(.3);
        }

        if (gamepad1.x) {

        }
        if (gamepad1.b) {
            intake.setPower(0);
        }
//
        if (gamepad1.right_bumper) {
            RHook.setPosition(1);
            LHook.setPosition(0);
        }
        if (gamepad1.left_bumper) {
            RHook.setPosition(.75);
            LHook.setPosition(.25);
        }
        if (gamepad1.left_trigger > .5) {
            RHook.setPosition(.4);
            LHook.setPosition(.4);
        }
        if (gamepad1.right_trigger > .5) {
            airplane.setPosition(0);
        }
            else {
            airplane.setPosition(.3);
        }
        if (gamepad1.dpad_up) {
            LHang.setPower(.2);
            RHang.setPower(.2);
        }
        else {
            LHang.setPower(0);
            RHang.setPower(0);
        }
        if (gamepad1.dpad_down) {
            LLock.setPosition(.1);
            RLock.setPosition(.1);
        }
        else {
            RLock.setPosition(0);
            LLock.setPosition(0);
        }
        if (gamepad2.a) {
           Lift.setTargetPosition(-1400);
           Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
           Lift.setPower(1);
           Door.setPosition(.3);
        }
        if (gamepad2.b) {
            Lift.setTargetPosition(-10);
            Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Lift.setPower(1);
        }
        if (gamepad2.dpad_down) {
            SLift.setPosition(.81);
            Pivot.setPosition(.7);
            Door.setPosition(.45);
        }
        if (gamepad2.dpad_up) {
            SLift.setPosition(.15);
            Pivot.setPosition(1);
        }
        if (gamepad2.dpad_right) {
            SLift.setPosition(.5);
        }

        if (gamepad2.right_bumper) {
            Door.setPosition(.4);
        }
        if (gamepad2.left_bumper) {
            Door.setPosition(.43);
        }
        // Door Open .45
        // Door Closed .3
        // Pivot down score .45
        // Pivot store .65
        // Pivot up score 1
        // SLift Placing posistion .15
        // SLift Store Posistion .75

        LiftCounts = Lift.getCurrentPosition();


        telemetry.addData("lift counts:", LiftCounts);

        telemetry.addData("ServoRpos", Rservopos);
        telemetry.addData("ServoLpos", Lservopos);

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addData("Color Sensor (Rev 3)", " %d %d %d", Math.round(colors.red * 10000), Math.round(colors.green * 10000), Math.round(colors.blue * 10000));
        telemetry.addData( "Colors (hsv)", "%d %d %d %d", Math.round(hsvValues[0] * 1), Math.round(hsvValues[1] * 1), Math.round(hsvValues[2] * 1), Math.round(colors.alpha * 10000));
        if ( colors.red * 10000 > 200 && colors.red * 10000 < 270) {
            pixcol = "Purple";
        }
        if (colors.red * 10000 < 100 && colors.red * 10000 > 50) {
            pixcol = "Green";
        }
        if (colors.red * 10000 < 350 && colors.red * 10000 > 270) {
            pixcol = "Yellow";
        }
        if (colors.red * 10000 > 400) {
            pixcol = "White";
        }
        telemetry.addData("Suspected Pixel Color", "%s", pixcol);
        telemetry.update();

        MecanumDrive.cartesian(Globals.robot,
                -leftStickY * .5,
                leftStickX * .5,
                rightStickX * .35);
    }
}
