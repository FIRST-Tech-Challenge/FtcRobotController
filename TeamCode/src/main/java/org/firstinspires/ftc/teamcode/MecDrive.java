//  _____                _           _       _  _    __  _____ _  _
// |  ___| __ ___   __ _| |__   ___ | |_ ___| || |  / /_|___ /| || |
// | |_ | '__/ _ \ / _` | '_ \ / _ \| __/ __| || |_| '_ \ |_ \| || |_
// |  _|| | | (_) | (_| | |_) | (_) | |_\__ \__   _| (_) |__) |__   _|
// |_|  |_|  \___/ \__, |_.__/ \___/ \__|___/  |_|  \___/____/   |_|
//                 |___/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drivebase.CenterStageDriveBase;
import org.firstinspires.ftc.teamcode.drivebase.StateM.FAILSAFELiftDownStateM;
import org.firstinspires.ftc.teamcode.drivebase.StateM.HangStateM;
import org.firstinspires.ftc.teamcode.drivebase.StateM.LiftDownStateM;
import org.firstinspires.ftc.teamcode.drivebase.StateM.LiftStateM;
import org.firstinspires.ftc.teamcode.drivebase.StateM.LiftStateM;
import org.firstinspires.ftc.teamcode.drivebase.StateM.StateMBase;
import org.firstinspires.ftc.teamcode.drivebase.StateM.StateMachine;
import org.firstinspires.ftc.teamcode.drivebase.StateM.LiftDownStateM;

import static org.firstinspires.ftc.teamcode.Globals.Door;
import static org.firstinspires.ftc.teamcode.Globals.Lift;
import static org.firstinspires.ftc.teamcode.Globals.Pivot;
import static org.firstinspires.ftc.teamcode.Globals.SLift;


@TeleOp
public class MecDrive extends LinearOpMode {

    // Drive Motors
    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx RL;
    private DcMotorEx RR;
    
    // Other Motors
    private       DcMotorEx intake;
//    public static DcMotorEx Lift;

    public static DcMotorEx RHang;
    public static DcMotorEx LHang;

    // Servos
    public static Servo RHook;
    public static Servo LHook;
    private       Servo intakeLift;
    public static Servo airplane;
   // public static Servo Door;
    public static Servo RLock;
    public static Servo LLock;
   // public static Servo Pivot;
   // public static Servo SLift;
//    private       Servo RCLaw;
//    private       Servo LCLaw;
    private float Rservopos;
    private float Lservopos;
    //    private       Servo RCLaw;
//    private       Servo LCLaw;

    // Local Variables
    private double leftStickX;
    private double leftStickY;
    private double rightStickX;
    private float  speedFactor;

    // Magic state machine thing
    public static boolean RESETME;

    // Sysinternals, dont touch
    CenterStageDriveBase centerStageDriveBase;
    TrackingWheelIntegrator trackingWheelIntegrator;

    // State Machine
    HangStateM HSM = new HangStateM();
    LiftStateM LSM = new LiftStateM();
    LiftDownStateM LDSM = new LiftDownStateM();
    FAILSAFELiftDownStateM FSLDSM = new FAILSAFELiftDownStateM();



    public void runOpMode() {

        // Drive Motors
        FL =            (DcMotorEx)     hardwareMap.get(DcMotorEx.class, "FL");
        FR =            (DcMotorEx)     hardwareMap.get(DcMotorEx.class, "FR");
        RL =            (DcMotorEx)     hardwareMap.get(DcMotorEx.class, "RL");
        RR =            (DcMotorEx)     hardwareMap.get(DcMotorEx.class, "RR");

        // Other Motors
        intake =        (DcMotorEx)     hardwareMap.get(DcMotorEx.class, "intake");
        Lift =          (DcMotorEx)     hardwareMap.get(DcMotorEx.class, "Lift");
        LHang =         (DcMotorEx)     hardwareMap.get(DcMotorEx.class, "LHang");
        RHang =         (DcMotorEx)     hardwareMap.get(DcMotorEx.class, "RHang");

        // Servos
        intakeLift =    (Servo)         hardwareMap.get(Servo.class, "intakeLift");
        airplane =      (Servo)         hardwareMap.get(Servo.class, "airplane");
        Door =          (Servo)         hardwareMap.get(Servo.class, "Door");
        RLock =         (Servo)         hardwareMap.get(Servo.class, "RLock");
        LLock =         (Servo)         hardwareMap.get(Servo.class, "LLock");
        Globals.Pivot =         (Servo)         hardwareMap.get(Servo.class, "Pivot");
        Globals.SLift =         (Servo)         hardwareMap.get(Servo.class, "SLift");
//        RCLaw =         (Servo)         hardwareMap.get(Servo.class, "RCLaw");
//        LCLaw =         (Servo)         hardwareMap.get(Servo.class, "LCLaw");
        RHook =         (Servo)         hardwareMap.get(Servo.class,  "RHook");
        LHook =         (Servo)         hardwareMap.get(Servo.class, "LHook");

        // Sysinit, don't touch
        centerStageDriveBase = new CenterStageDriveBase();
        centerStageDriveBase.init(hardwareMap);
        centerStageDriveBase.enablePID();
        Globals.robot=centerStageDriveBase;
        Globals.driveBase=centerStageDriveBase;
        Globals.trackingWheelIntegrator = trackingWheelIntegrator;
        Globals.opMode = this;
        Globals.robot.enableBrake(true);
        trackingWheelIntegrator = new TrackingWheelIntegrator();

        // Initial Servo positions
        Rservopos = 1;
        Lservopos = -1;

        // Run to position
        Lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Lift.setTargetPosition(1);
        Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Telemetry initialisation
        telemetry.setMsTransmissionInterval(20);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();




        while (opModeIsActive()) {

            if (RESETME) {
                HSM.reset();
                LSM.reset();
                LDSM.reset();
                FSLDSM.reset();
                RESETME = false;
            }

            if (gamepad1.dpad_up) {
                HSM.runIteration();
                runGamepad();
            }
            if (gamepad2.a) {
                LSM.runIteration();
                runGamepad();
            }
            if (gamepad2.b) {
                LDSM.runIteration();
                runGamepad();
            }
            if (gamepad2.dpad_down) {
                FSLDSM.runIteration();
                runGamepad();
            }
            runGamepad();
        }

    }

    void runGamepad() {

        // Default speed, change with care
        speedFactor = (float) .5;

        // Map stick values to local variables
        leftStickX = gamepad1.left_stick_x;
        leftStickY = gamepad1.left_stick_y;
        rightStickX = gamepad1.right_stick_x;


        if (gamepad1.a) {
            intakeLift.setPosition(.5);
            intake.setPower(1);
            Door.setPosition(.85);
        }

        if (gamepad2.dpad_left) {
            speedFactor = (float) .1;
        }

        if (gamepad2.dpad_right) {
            speedFactor = (float) 1;
        }

        if (gamepad1.y) {
            intakeLift.setPosition(.3);
        }
        // 0 = l0 r1 = neuteral
        // .5 = half way for ap
        // 1 = l1 r0 = all the way

        if (gamepad1.b) {
            intake.setPower(0);
        }
        if (gamepad1.right_bumper) {
            SLift.setPosition(.9);
            Pivot.setPosition(.7);
        }
        if (gamepad1.left_bumper) {
           SLift.setPosition(.15);
           Pivot.setPosition(.8);
        }

//        if (gamepad2.right_stick_button) {
//            LLock.setPosition(.0);
//            RLock.setPosition(.1);
//        }
        // hang lock locked = .1
        // hang lock freed = 0

//        if (gamepad2.a) {
//           Lift.setTargetPosition(-1400);
//           Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//           Lift.setPower(1);
//         //  Door.setPosition(.3);
//        }
//        if (gamepad2.b) {
//            Lift.setTargetPosition(-10);
//            Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            Lift.setPower(1);
//        }
//        if (gamepad2.dpad_down) {
//            SLift.setPosition(.81);
//            Pivot.setPosition(.7);
//          //  Door.setPosition(.45);
//        }
        if (gamepad2.right_bumper) {
            Door.setPosition(.8);
        }
        if (gamepad2.x) {
            Door.setPosition(.94);
        }
        if (gamepad2.y) {
            Door.setPosition(.95);
        }
        if (gamepad2.left_bumper) {
            Door.setPosition(1);
        }
        if (gamepad2.left_trigger > .5) {
            Lift.setTargetPosition(-867);
            Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Lift.setPower(1);
        }
        if (gamepad2.right_trigger > .5) {
            Lift.setTargetPosition(-1400);
            Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Lift.setPower(1);
        }
        if (gamepad1.dpad_left) {
            RHook.setPosition(.75);
            LHook.setPosition(.25);
        }
        if (gamepad1.dpad_right) {
            airplane.setPosition(0);
        }
        else airplane.setPosition(.3);

        if (gamepad2.right_stick_button) {
            LLock.setPosition(.6);
            RLock.setPosition(.5);
        }
        if (gamepad2.left_stick_button) {
            LLock.setPosition(.1);
            RLock.setPosition(.9);
        }

//        if (gamepad2.right_bumper) {
////            RHook.setPosition(.6);
////            LHook.setPosition(.4);
//            airplane.setPosition(0);
//        }
//        else if (!gamepad2.dpad_up && !gamepad2.right_bumper) {
//            airplane.setPosition(.3);
//        }
//        if (gamepad1.dpad_right) {
//            airplane.setPosition(0);
//        }
//            else {
//                airplane.setPosition(.3);
//            }

        if (gamepad1.right_trigger > .5) {
            RHook.setPosition(1);
            LHook.setPosition(0);
            LHang.setPower(-.3);
            RHang.setPower(-.3);
        }
        else if (!gamepad1.dpad_up && (gamepad1.right_trigger > .5) && !gamepad1.dpad_down) {
            LHang.setPower(0);
            RHang.setPower(0);
        }
        if (gamepad1.left_trigger > .5) {
            RHook.setPosition(1);
            LHook.setPosition(0);

        }
        if (gamepad1.dpad_down) {
            RHook.setPosition(0);
            LHook.setPosition(1);
            LHang.setPower(-1);
            RHang.setPower(-1);
        }
        else if (!gamepad1.dpad_up && (gamepad1.right_trigger > .5) && !gamepad1.dpad_down) {
            LHang.setPower(0);
            RHang.setPower(0);
        }
//        if (gamepad1.left_bumper) {
//            RHook.setPosition(1);
//            LHook.setPosition(0);
//        }
//        if (gamepad1.right_bumper) {
//            RHook.setPosition(.5);
//            LHook.setPosition(.5);
//            intakeLift.setPosition(.5);
//        }
//        if (gamepad2.dpad_left) {
//            SLift.setPosition(.15);
//            Pivot.setPosition(.8);
//            Door.setPosition(1);
//        }
      //  if (gamepad1.left_trigger > .5) {
        //   RCLaw.setPosition(1);
          // LCLaw.setPosition(0);
       // }
       // if (gamepad1.right_trigger > .5) {
         //   RCLaw.setPosition(.65);
          //  LCLaw.setPosition(.35);
       // }


        // Door Open .45
        // Door Closed 1
        // Pivot down score .45
        // Pivot store .65
        // Pivot up score 1
        // SLift Placing posistion .15
        // SLift Store Posistion .75

//        LiftCounts = Lift.getCurrentPosition();
//
//        telemetry.addData("lift counts:", LiftCounts);

//        // Gamepad Mappings
//        if (gamepad1.a)                     { intakeLift.setPosition(.5); intake.setPower(1); }
//        if (gamepad1.dpad_left)             { speedFactor = (float) .1; }
//        if (gamepad1.dpad_right)            { speedFactor = (float) 1; }
//        if (gamepad1.y)                     { intakeLift.setPosition(.3); }
//        if (gamepad1.b)                     { intake.setPower(0); }
//        if (gamepad2.right_stick_button)    { LLock.setPosition(.0); RLock.setPosition(.1); }
//        if (gamepad2.a)                     { Lift.setTargetPosition(-1400); Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION); Lift.setPower(1); Door.setPosition(.3); }
//        if (gamepad2.b)                     { Lift.setTargetPosition(-10); Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION); Lift.setPower(1); }
//        if (gamepad2.x)                     { Door.setPosition(.23); }
//        if (gamepad2.y)                     { Door.setPosition(.4); }
//        if (gamepad2.dpad_down)             { SLift.setPosition(.81); Pivot.setPosition(.7); Door.setPosition(.45); }
//        if (gamepad2.dpad_up)               { SLift.setPosition(.15); Pivot.setPosition(1); }
//        if (gamepad2.dpad_right)            { SLift.setPosition(.5); }
//        if (gamepad2.left_bumper)           { RHook.setPosition(.75); LHook.setPosition(.25); }
//        if (gamepad2.left_trigger > .5)     { RHook.setPosition(0); LHook.setPosition(1); LHang.setPower(1); RHang.setPower(1); } else { LHang.setPower(0); RHang.setPower(0); }
//        if (gamepad2.dpad_left)             { Pivot.setPosition(.8); Door.setPosition(0); }
//        if (gamepad1.left_trigger > .5)     { RCLaw.setPosition(1); LCLaw.setPosition(0); }
//        if (gamepad1.right_trigger > .5)    { RCLaw.setPosition(.65); LCLaw.setPosition(.35); }

        // Telemetry
        telemetry.addData("lift counts:", Lift.getCurrentPosition());
        telemetry.addData("ServoRpos", Rservopos);
        telemetry.addData("ServoLpos", Lservopos);
        telemetry.update();

        // Drive
        MecanumDrive.cartesian(Globals.robot, -leftStickY * speedFactor, leftStickX * speedFactor, rightStickX * speedFactor);
    }
}

// VALUE INDEX

// Door Servo Positions
// Door Open .45
// Door Closed .3

// Pivot Servo Positions
// Pivot down score .45
// Pivot store .65
// Pivot up score 1

// SLift Positions
// SLift Placing posistion .15
// SLift Store Posistion .75

// Intake Lift Positions
// 0 = l0 r1 = neuteral
// .5 = half way for ap
// 1 = l1 r0 = all the way

// Hang positions
// hang lock locked = .1
// hang lock freed = 0