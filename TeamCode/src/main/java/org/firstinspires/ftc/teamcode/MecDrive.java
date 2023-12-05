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
    private Servo RHook;
    private Servo LHook;

    private float Rservopos;
    private float Lservopos;

    private double leftStickX;
    private double leftStickY;
    private double rightStickX;

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
            Rservopos = (float) (Rservopos + 0.01);
            if (Rservopos > 1){
                Rservopos = 0.99F;
            }
            RHook.setPosition(Rservopos);
            //LHook.setPosition(Lservopos);
        }

        if (gamepad1.b) {
            Rservopos = (float) (Rservopos - 0.01);
            if (Rservopos < 0) {
                Rservopos = 0;
            }
            RHook.setPosition(Rservopos);
            //LHook.setPosition(Lservopos);
        }

        if (gamepad1.x) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

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
                -leftStickY * speedFactor,
                leftStickX * speedFactor,
                rightStickX * speedFactor * .75);
    }
}
