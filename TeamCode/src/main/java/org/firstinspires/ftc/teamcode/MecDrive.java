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

    // Drive Motors
    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx RL;
    private DcMotorEx RR;

    // Misc. Motors
    private DcMotorEx intake;
    private DcMotorEx Lift;
    private DcMotorEx RHang;
    private DcMotorEx LHang;

    //Misc. Servos
    private Servo RHook;
    private Servo LHook;
    private Servo intakeLift;
    private Servo airplane;

    //Lift Servos
    private Servo Door;
    private Servo RLock;
    private Servo LLock;
    private Servo Pivot;
    private Servo SLift;

    // Various Variables (don't touch)
    private float Rservopos;
    private float Lservopos;
    private double leftStickX;
    private double leftStickY;
    private double rightStickX;
    private float speedFactor;

    // DONT REMOVE
    CenterStageDriveBase centerStageDriveBase;
    TrackingWheelIntegrator trackingWheelIntegrator;

    public void runOpMode() {

        trackingWheelIntegrator = new TrackingWheelIntegrator();

        // CONFIGURATION AND HARDWAREMAP

        // Drive Motor Setup
        FL =          (DcMotorEx) hardwareMap.get(DcMotorEx.class, "FL");
        FR =          (DcMotorEx) hardwareMap.get(DcMotorEx.class, "FR");
        RL =          (DcMotorEx) hardwareMap.get(DcMotorEx.class, "RL");
        RR =          (DcMotorEx) hardwareMap.get(DcMotorEx.class, "RR");

        // Other Motors
        intake =      (DcMotorEx) hardwareMap.get(DcMotorEx.class, "intake");
        Lift =        (DcMotorEx) hardwareMap.get(DcMotorEx.class, "Lift");
        LHang =       (DcMotorEx) hardwareMap.get(DcMotorEx.class, "LHang");
        RHang =       (DcMotorEx) hardwareMap.get(DcMotorEx.class, "RHang");

        // Servos
        intakeLift =  (Servo)  hardwareMap.get(Servo.class, "intakeLift");
        airplane =    (Servo)  hardwareMap.get(Servo.class, "airplane");
        Door =        (Servo)  hardwareMap.get(Servo.class, "Door");
        RLock =       (Servo)  hardwareMap.get(Servo.class, "RLock");
        LLock =       (Servo)  hardwareMap.get(Servo.class, "LLock");
        Pivot =       (Servo)  hardwareMap.get(Servo.class, "Pivot");
        SLift =       (Servo)  hardwareMap.get(Servo.class, "SLift");
        RHook =       (Servo)  hardwareMap.get(Servo.class, "RHook");
        LHook =       (Servo)  hardwareMap.get(Servo.class, "LHook");


        //colorSensor= (NormalizedColorSensor) hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        // Sys Internals (DONT TOUCH)
        centerStageDriveBase = new CenterStageDriveBase();
        centerStageDriveBase.init(hardwareMap);
        centerStageDriveBase.enablePID();
        Globals.robot=centerStageDriveBase;
        Globals.driveBase=centerStageDriveBase;
        Globals.trackingWheelIntegrator = trackingWheelIntegrator;
        Globals.opMode = this;
        Globals.robot.enableBrake(true);

        // Servo initial positioning
        Rservopos = 1;
        Lservopos = -1;

        // Run to position stuff
        Lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Lift.setTargetPosition(1);
        Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Telemetry
        telemetry.setMsTransmissionInterval(20);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // DONT REMOVE
        waitForStart();

        // DONT REMOVE
        while (opModeIsActive()) {
            runGamepad();
        }




    }

    public void Hang(double Rightservpos, double Leftservpos, double endpos, double speed) {
        RHook.setPosition(Rightservpos);
        LHook.setPosition(Leftservpos);
        while (LHook.getPosition() != endpos) {
            LHang.setPower(speed);
            RHang.setPower(speed);
        }
    }

    void runGamepad() {

        // This is for the default speed, change it with care
        speedFactor = (float) .5;

        // To sync the gamepad1 sticks with local variables
        leftStickX = gamepad1.left_stick_x;
        leftStickY = gamepad1.left_stick_y;
        rightStickX = gamepad1.right_stick_x;

        // All controller mappings go here
        if (gamepad1.a)                  { intakeLift.setPosition(.5); intake.setPower(1); }
        if (gamepad1.dpad_left)          { speedFactor = (float) .1; }
        if (gamepad1.dpad_right)         { speedFactor = (float) 1; }
        if (gamepad1.y)                  { intakeLift.setPosition(.3); }
        if (gamepad1.b)                  { intake.setPower(0); }
        if (gamepad1.right_bumper)       { Hang(1, 0, 0, 1); }
        if (gamepad1.left_bumper)        { Hang(0.7, 0.3, 0.25, -0.2); }
        if (gamepad1.right_trigger > .5) { Hang(0.4, 0.4, 0.4, .3); }
        if (gamepad1.left_trigger > .5)  { airplane.setPosition(0); } else { airplane.setPosition(.3); }
        if (gamepad1.dpad_up)            { LHang.setPower(.3); RHang.setPower(.3); } else { LHang.setPower(0); RHang.setPower(0); }
        if (gamepad1.dpad_down)          { LLock.setPosition(.1); RLock.setPosition(.1); } else { RLock.setPosition(0); LLock.setPosition(0); }
        if (gamepad2.a)                  { Lift.setTargetPosition(-1400); Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION); Lift.setPower(1); Door.setPosition(.3); }
        if (gamepad2.b)                  { Lift.setTargetPosition(-10); Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION); Lift.setPower(1); }
        if (gamepad2.dpad_down)          { SLift.setPosition(.81); Pivot.setPosition(.7); Door.setPosition(.45); }
        if (gamepad2.dpad_up)            { SLift.setPosition(.15); Pivot.setPosition(1); }
        if (gamepad2.dpad_right)         { SLift.setPosition(.5); }
        if (gamepad1.dpad_left)          { Pivot.setPosition(.8); Door.setPosition(0); }
        if (gamepad2.right_bumper)       { Door.setPosition(.43); }
        if (gamepad2.left_bumper)        { Door.setPosition(.45); }

        // SERVO VALUES INDEX
        // Door Open .45
        // Door Closed .3
        // Pivot down score .45
        // Pivot store .65
        // Pivot up score 1
        // SLift Placing Position .15
        // SLift Store Position .75

        // Telemetry
        telemetry.addData("lift counts:", Lift.getCurrentPosition());
        telemetry.addData("ServoRpos", Rservopos);
        telemetry.addData("ServoLpos", Lservopos);
        telemetry.update();

        // Applying stick positions to the wheels, don't remove
        MecanumDrive.cartesian(Globals.robot,
                -leftStickY * speedFactor,
                leftStickX * speedFactor,
                rightStickX * speedFactor);
    }
}