package org.firstinspires.ftc.teamcode;

import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drivebase.CenterStageDriveBase;

@Autonomous(preselectTeleOp = "MecDrive")
public class MainAuto extends LinearOpMode {

    TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();

    CenterStageDriveBase centerStageDriveBase;

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

    @Override
    public void runOpMode() throws InterruptedException {

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

        centerStageDriveBase = new CenterStageDriveBase();
        centerStageDriveBase.init(hardwareMap);
        centerStageDriveBase.resetEncoders();
        centerStageDriveBase.enableBrake(true);
        centerStageDriveBase.enablePID();
        Globals.robot=centerStageDriveBase;
        Globals.driveBase=centerStageDriveBase;
        Globals.opMode = this;
        Globals.trackingWheelIntegrator = trackingWheelIntegrator;

        telemetry.setMsTransmissionInterval(20);



    }
}
