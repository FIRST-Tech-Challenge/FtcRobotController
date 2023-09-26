package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.drivebase.CenterStageDriveBase;


@TeleOp
public class MecDrive extends LinearOpMode {

    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx RL;
    private DcMotorEx RR;

    private double leftStickX;
    private double leftStickY;
    private double rightStickX;

    NormalizedColorSensor colorSensor;

    CenterStageDriveBase centerStageDriveBase;
    TrackingWheelIntegrator trackingWheelIntegrator;

    float[] hsvValues = new float[3];

    public void runOpMode() {

        trackingWheelIntegrator = new TrackingWheelIntegrator();

        FL= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "FL");
        FR= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "FR");
        RL= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "RL");
        RR= (DcMotorEx) hardwareMap.get(DcMotorEx.class, "RR");
        colorSensor= (NormalizedColorSensor) hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        centerStageDriveBase = new CenterStageDriveBase();
        centerStageDriveBase.init(hardwareMap);
        centerStageDriveBase.enablePID();
        Globals.robot=centerStageDriveBase;
        Globals.driveBase=centerStageDriveBase;
        Globals.trackingWheelIntegrator = trackingWheelIntegrator;
        Globals.opMode = this;
        Globals.robot.enableBrake(true);

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

        if(gamepad1.x) {
            FL.setPower(1);
        }
        if(gamepad1.y) {
            FR.setPower(1);
        }
        if(gamepad1.a) {
            RL.setPower(1);
        }
        if(gamepad1.b) {
            RR.setPower(1);
        }

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addData("Color Sensor (Rev 3)", "%.3f %.3f %.3f ", colors.red, colors.green, colors.blue);

        MecanumDrive.cartesian(Globals.robot,
                -leftStickY * .5,
                leftStickX * .5,
                rightStickX * .35);
    }
}
