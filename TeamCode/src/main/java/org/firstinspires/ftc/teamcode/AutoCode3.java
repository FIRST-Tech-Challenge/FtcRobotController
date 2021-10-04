package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.core.movement.old.ChassisMovementCode;

import java.util.concurrent.TimeUnit;
//At this point it's almost entirely created by Ethan
//38 inches between robots
@Autonomous
public class AutoCode3 extends LinearOpMode {

    private Blinker Control_Hub;
    private Blinker expansion_Hub_2;
    ElapsedTime servoTimer = new ElapsedTime();
    ElapsedTime GeneralTimer = new ElapsedTime();

    private double RotationPreset = 0;

    private enum OperState {
        PrepClose,
        TimeClose,
        PrepLift,
        TimeLift,
        FIRSTMOVE,
        PrepFMove,
        Rotate,
        PrepGetCloser,
        GetCloser,
        PrepUnGetCloser,
        UnGetCloser,
        UnRotate,
        PrepMoveToLine,
        MoveToLine,
        DECIDE,
        MEASURE,
        PrepGoToTargetZone,
        PREPB,
        PREPC,
        GoToTargetZone,
        PrepDrop,
        Drop,
        PrepStrafeLeft,
        PrepLaunchPark,
        PrepMoveToShooting,
        MoveToPowerShots,
        StrafeLeft,
        LaunchPark,
        MoveToShooting,
        NextLocation,
        SpinAround,
        Launch,
        Delayer,
        Reload
    }

    private enum Menu {
        StartLocation,
        DelayAndGo,
        ButtonWaiter0,
        ButtonWaiter1,
        Powershots,
        ButtonWaiter2,
        OnlyPark,
        ButtonWaiter3,
        Goals,
        ButtonWaiter4,
        AreYouMoving,
        ButtonWaiter5,
        CheckForInvalid,
        Save,
        AskIfDone,
        Redo,
        ButtonWaiter6
    }

    @Override
    public void runOpMode() {
        LauncherCode.Launcher launcher = new LauncherCode.Launcher();
        LifterCode.Lifter lift = new LifterCode.Lifter();
        ChassisMovementCode.Chassis chassis = new ChassisMovementCode.Chassis();
        AutoCode3.OperState driveOpState = AutoCode3.OperState.PrepClose;
        AutoCode3.Menu menu = AutoCode3.Menu.StartLocation;
        DistanceSensorClass.RingClass ring = new DistanceSensorClass.RingClass();
        Grabber grabber = new Grabber();
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        lift.LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        grabber.GrabberLeft = hardwareMap.get(Servo.class, "GrabberLeft");
        grabber.GrabberRight = hardwareMap.get(Servo.class, "GrabberRight");
        launcher.LaunchMotor = hardwareMap.get(DcMotor.class, "LaunchMotor");
        launcher.LaunchServo = hardwareMap.get(Servo.class, "LaunchServo");
        chassis.imu = hardwareMap.get(BNO055IMU.class, "imu");
        chassis.front_left_wheel = hardwareMap.get(DcMotor.class, "front left wheel");
        chassis.front_right_wheel = hardwareMap.get(DcMotor.class, "front right wheel");
        chassis.back_left_wheel = hardwareMap.get(DcMotor.class, "back left wheel");
        chassis.back_right_wheel = hardwareMap.get(DcMotor.class, "back right wheel");
        ring.DistanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        chassis.imu.initialize(parameters);
        double drivePreset = 0;
        double strafePreset = 0;
        double rotationGoal = chassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double originalRotation = rotationGoal;
        //menu variables
        boolean IsMenuDone = false;
        boolean OnRed = true; //true is on red, false is on blue.
        boolean DoneMeasuring = false;
        int StartLocation = 0; //1 is Blue 1, 2 is Blue 2, 3 is Red 1, 4 is Red 2.
        int Powershots = 0; //1 is yes, 2 is no.
        int ShootGoals = 0; //1 is yes, 2 is no.
        int OnlyPark = 0; //1 is yes, 2 is no.
        int AreYouMoving = 0; //1 is yes, 2 is no.
        int Save = 0; //1 is yes, 2 is no.
        int DelayAndGo = 0; //1 is yes, 2 is no.
        //23.5 Inches Between the strips.

        //All Constants For All Moves
        double moverightstrafe = 0;
        double initrotation = 0;
        double rotmove = 0;
        double closermove = -5.5;
        double targetdrive = 0;
        double targetstrafe = 0;
        double bdrive = 0;
        double bstrafe = 0;
        double cdrive = 0;
        double cstrafe = 0;
        double shootdrive = 0;
        double shootstrafe = 0;
        double strafeslightleft = -4.75;
        double targetdrivespeed = 1;
        int launchCount = 0;
        int ringCount = 0;
        double launchpower = 1;
        boolean onodd = false;
        double shootspeed = 0.4;
        double launchparkdrive = 13;
        ElapsedTime MultipleUsesTimer = new ElapsedTime();
        chassis.SetRotation(chassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        while (!IsMenuDone) {
            switch (menu) {
                case StartLocation:
                    telemetry.addLine("Blue1(A) Blue2(B) Red1(X) Red2(Y)?");
                    telemetry.update();
                    if (gamepad1.a) {
                        StartLocation = 1;
                        OnRed = false;
                        menu = AutoCode3.Menu.ButtonWaiter0;
                    } else if (gamepad1.b) {
                        StartLocation = 2;
                        OnRed = false;
                        menu = AutoCode3.Menu.ButtonWaiter0;
                    } else if (gamepad1.x) {
                        StartLocation = 3;
                        OnRed = true;
                        menu = AutoCode3.Menu.ButtonWaiter0;
                    } else if (gamepad1.y) {
                        StartLocation = 4;
                        OnRed = true;
                        menu = AutoCode3.Menu.ButtonWaiter0;
                    }
                    break;
                case ButtonWaiter0:
                    if ((StartLocation == 1 && !gamepad1.a) || (StartLocation == 2 && !gamepad1.b) || (StartLocation == 3 && !gamepad1.x) || (StartLocation == 4 && !gamepad1.y) ) { menu = AutoCode3.Menu.DelayAndGo; }
                    break;
                case DelayAndGo:
                    telemetry.addLine("Are you doing the Wobble Goals? If you aren't, then the robot will delay for 11 seconds and then go straight to the launch location. Yes (Y) No (X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        DelayAndGo = 2;
                        menu = AutoCode3.Menu.ButtonWaiter1;
                    } else if (gamepad1.y) {
                        DelayAndGo = 1;
                        menu = AutoCode3.Menu.ButtonWaiter1;
                    }
                    break;
                case ButtonWaiter1:
                    if ((DelayAndGo == 2) && (!gamepad1.x)) {
                        menu = AutoCode3.Menu.Powershots;
                    } else if ((DelayAndGo == 1) && (!gamepad1.y)) {
                        menu = AutoCode3.Menu.Powershots;
                    }
                    break;
                case Powershots:
                    telemetry.addLine("Shoot Powershots? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        Powershots = 2;
                        menu = AutoCode3.Menu.ButtonWaiter2;
                    } else if (gamepad1.y) {
                        Powershots = 1;
                        menu = AutoCode3.Menu.ButtonWaiter2;
                    }
                    break;
                case ButtonWaiter2:
                    if ((Powershots == 2 && !gamepad1.x)) {
                        menu = AutoCode3.Menu.Goals;
                    } else if ((Powershots == 1 && !gamepad1.y)) {
                        menu = AutoCode3.Menu.CheckForInvalid;
                    }
                    break;
                case Goals:
                    telemetry.addLine("Shoot into the top goal? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        ShootGoals = 2;
                        menu = AutoCode3.Menu.ButtonWaiter3;
                    } else if (gamepad1.y) {
                        ShootGoals = 1;
                        menu = AutoCode3.Menu.ButtonWaiter3;
                    }
                    break;
                case ButtonWaiter3:
                    if ((ShootGoals == 2) & (!gamepad1.x)) {
                        menu = AutoCode3.Menu.OnlyPark;
                    } else if ((ShootGoals == 1) & (!gamepad1.y)) {
                        menu = AutoCode3.Menu.CheckForInvalid;
                    }
                    break;
                case OnlyPark:
                    telemetry.addLine("Would you like to park in the corner? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        OnlyPark = 2;
                        menu = AutoCode3.Menu.ButtonWaiter4;
                    } else if (gamepad1.y) {
                        OnlyPark = 1;
                        menu = AutoCode3.Menu.ButtonWaiter4;
                    }
                    break;
                case ButtonWaiter4:
                    if ((OnlyPark == 2) & (!gamepad1.x)) {
                        menu = AutoCode3.Menu.AreYouMoving;
                    } else if ((OnlyPark == 1) & (!gamepad1.y)) {
                        menu = AutoCode3.Menu.CheckForInvalid;
                    }
                    break;
                case AreYouMoving:
                    telemetry.addLine("Would you like to stay put? Yes(Y) No(X) No will cause an error and kick you back to the beginning.");
                    telemetry.update();
                    if (gamepad1.x) {
                        AreYouMoving = 2;
                        menu = AutoCode3.Menu.ButtonWaiter5;
                    } else if (gamepad1.y) {
                        AreYouMoving = 1;
                        menu = AutoCode3.Menu.ButtonWaiter5;
                    }
                    break;
                case ButtonWaiter5:
                    if ((AreYouMoving == 2) & (!gamepad1.x)) {
                        menu = AutoCode3.Menu.CheckForInvalid;
                    } else if ((AreYouMoving == 1) & (!gamepad1.y)) {
                        menu = AutoCode3.Menu.CheckForInvalid;
                    }
                    break;
                case CheckForInvalid:
                    if (Powershots == 2 & ShootGoals == 2 & OnlyPark == 2 & AreYouMoving == 2) { menu = AutoCode3.Menu.Redo; }
                    else { menu = AutoCode3.Menu.AskIfDone; }
                    break;
                case AskIfDone:
                    telemetry.addLine("Would you like to save these changes? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        Save = 2;
                        menu = AutoCode3.Menu.ButtonWaiter6;
                    } else if (gamepad1.y) {
                        Save = 1;
                        menu = AutoCode3.Menu.ButtonWaiter6;
                    }
                    break;
                case ButtonWaiter6:
                    if ((Save == 2) & (!gamepad1.x)) {
                        menu = AutoCode3.Menu.Redo;
                    } else if ((Save == 1) & (!gamepad1.y)) {
                        menu = AutoCode3.Menu.Save;
                    }
                    break;
                case Redo:
                    StartLocation = 0;
                    Powershots = 0;
                    ShootGoals = 0;
                    OnlyPark = 0;
                    AreYouMoving = 0;
                    Save = 0;
                    DelayAndGo = 0;
                    OnRed = true;
                    strafeslightleft = -4.75;
                    driveOpState = AutoCode3.OperState.PrepClose;
                    menu = AutoCode3.Menu.StartLocation;
                    DoneMeasuring = false;
                    break;
                case Save:
                    if (DelayAndGo == 2) {
                        driveOpState = AutoCode3.OperState.Delayer;
                        DoneMeasuring = true;
                    }
                    else { shootspeed = 1.33; }
                    if (Powershots == 1 && !OnRed) { strafeslightleft = -strafeslightleft; }
                    if (Powershots == 1) { launchpower = 0.939; }
                    if (StartLocation == 1 || StartLocation == 3) { onodd = true; }
                    if (onodd) {
                        rotmove = -25.5;
                        initrotation = 45;
                    } else {
                        initrotation = -67.3;
                        rotmove = -22.5;
                        closermove = -4.78;
                    }

                    switch (StartLocation) {
                        case 1:
                            if (DelayAndGo == 2) {
                                if (Powershots == 1) {
                                    shootdrive = 61.87;
                                    shootstrafe = 40;
                                }
                                
                                if (ShootGoals == 1) {
                                    shootdrive = 56.5;
                                    shootstrafe = 22.5;
                                    shootdrive -= 8.5;
                                    launchparkdrive += 8.5;
                                }
                            }
                            break;

                        case 2:
                            if (DelayAndGo == 2) {
                                if (Powershots == 1) {
                                    shootdrive = 61.87;
                                    shootstrafe = 14.5;
                                }
                                
                                if (ShootGoals == 1) {
                                    shootdrive = 56.5;
                                    shootstrafe = -4;
                                    shootdrive -= 8.5;
                                    launchparkdrive += 8.5;
                                }
                            }
                            break;

                        case 3:
                            if (DelayAndGo == 2) {
                                if (Powershots == 1) {
                                    shootdrive = 61.87;
                                    shootstrafe = -.5;
                                }
                                
                                if (ShootGoals == 1) {
                                    shootdrive = 56.5;
                                    shootstrafe = 22.5;
                                    shootdrive -= 8.5;
                                    launchparkdrive += 8.5;
                                }
                            }
                            break;

                        case 4:
                            if (DelayAndGo == 2) {
                                if (Powershots == 1) {
                                    shootdrive = 61.87;
                                    shootstrafe = -24;
                                }
                                
                                if (ShootGoals == 1) {
                                    shootdrive = 56.5;
                                    shootstrafe = -4;
                                    shootdrive -= 8.5;
                                    launchparkdrive += 8.5;
                                }
                            }
                            break;
                    }
                    if (Powershots == 1) { shootdrive -= 1.32; }
                    telemetry.addLine("Choices have been saved. You may now tell the ref you are ready.");
                    telemetry.update();
                    IsMenuDone = true;
                    break;
            }
        }

        waitForStart();
        servoTimer.reset();
        launcher.Reload();
        originalRotation = chassis.zAngle;
        initrotation = -initrotation;
/*                    if (chassis.MoveToLocation() == true) {
                        telemetry.addLine("done");
                    }
*/
        while (opModeIsActive()) {
            chassis.SetRotation(chassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("how close", chassis.zAngle - (originalRotation+initrotation));
            launcher.LauncherRun(launchpower);
            if (!DoneMeasuring) { ring.MeasureDistance(onodd); }
            telemetry.addData("where you are in strafe", Math.abs(chassis.strafePreset - chassis.trueStrafe));
            telemetry.addData("driveopstate", driveOpState);
            telemetry.addData("IMPORTANT, DRIVE PRESET", chassis.drivePreset);
            telemetry.addData("IMPORTANT, STRAFE PRESET", chassis.strafePreset);
            telemetry.addData("Launch Count", launchCount);
            telemetry.addData("Ring Count", ringCount);
            telemetry.addData("og rotation", originalRotation);
            telemetry.addData("zangle", chassis.zAngle);
            switch (driveOpState) {
               /* case FIRSTMOVE:
                    telemetry.addLine("FIRSTMOVE");
                    originalRotation = chassis.zAngle;
                    if (servoTimer.time() >= 2) { driveOpState = AutoCode3.OperState.PREPMOVEANDLIFT; }
                    break;
               */
                case PrepClose:
                    grabber.Close();
                    GeneralTimer.reset();
                    driveOpState = OperState.TimeClose;
                    break;
                case TimeClose:
                    if (GeneralTimer.time(TimeUnit.SECONDS) >= 0.31) { driveOpState = OperState.PrepFMove; }
                    break;
                case PrepLift:
                    lift.MoveLift(0.31);
                    GeneralTimer.reset();
                    driveOpState = OperState.TimeLift;
                    break;
                case TimeLift:
                    if (GeneralTimer.time(TimeUnit.SECONDS) >= 0.16) {
                        driveOpState = OperState.PrepMoveToLine;
                        lift.Stop();
                    }
                    break;
                case PrepFMove:
                    lift.MoveLift(0);
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(rotmove, 1, 0, .4, chassis.zAngle);
                    driveOpState = OperState.FIRSTMOVE;
                    break;
                case FIRSTMOVE:
                    if (chassis.MoveToLocation()) { driveOpState = AutoCode3.OperState.Rotate; }
                    break;
                case Rotate:
                    if ((Math.abs(chassis.zAngle - (originalRotation+initrotation)) >= 1.5)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, (originalRotation+initrotation),2));
                        chassis.Drive();
                        chassis.SetAxisMovement();
                        chassis.ZeroEncoders();
                        chassis.SetAxisMovement();
                        rotationGoal = chassis.zAngle;
                    }
                    else {
                        chassis.SetMotors(0,0,0);
                        chassis.Drive();
                        driveOpState = OperState.PrepGetCloser;
                    }
                    break;
                case PrepGetCloser:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(closermove, 0.64, 0, .4, chassis.zAngle);
                    driveOpState = OperState.GetCloser;
                    break;
                case GetCloser:
                    if (chassis.MoveToLocation()) {
                        driveOpState = OperState.MEASURE;
                        MultipleUsesTimer.reset();
                    }
                    break;
                case Delayer:
                    if (servoTimer.time() >= 13.5) {
                        driveOpState = AutoCode3.OperState.PrepMoveToShooting;
                        servoTimer.reset();
                    }
                    break;
/*
                case PREPMOVEANDLIFT:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(-19., 0.5, 0, 0.000000000000001, chassis.zAngle);
                    servoTimer.reset();
                    driveOpState = OperState.MOVEANDLIFT;
                    break;

                case MOVEANDLIFT:
                    telemetry.addLine("newsecondmoveE");
                    telemetry.addData("driveStrafe", drivePreset);
                    telemetry.addData("strafePreset", strafePreset);
                    telemetry.addData("rotate", chassis.rotation);
                    telemetry.addData("drivevalue", chassis.trueDrive);
                    telemetry.addData("strafevalue", chassis.trueStrafe);
                    telemetry.addData("drive", chassis.drive);
                    telemetry.addData("strafe", chassis.strafe);
                    telemetry.addData("back left wheel", chassis.backLeft);
                    telemetry.addData("back right wheel", chassis.backRight);
                    telemetry.addData("front right wheel", chassis.frontRight);
                    telemetry.addData("front left wheel", chassis.frontLeft);
                    telemetry.addData("firstSignumRotate", Math.signum(rotationGoal - chassis.zAngle));
                    telemetry.addData("firstSignumStrafe", Math.signum(strafePreset - chassis.trueStrafe));
                    telemetry.addData("firstSignumDrive", Math.signum(drivePreset - chassis.trueDrive));
                    telemetry.addData("Math.maxRotate", (Math.max(0.2, Math.abs((rotationGoal - chassis.zAngle) / 180))));
                    telemetry.addData("Math.maxStrafe", (Math.max(0.2, Math.abs((strafePreset - chassis.trueStrafe) / strafePreset))));
                    telemetry.addData("Math.maxDrive", (Math.max(0.2, Math.abs((drivePreset - chassis.trueDrive) / drivePreset))));
                    if (chassis.MoveToLocation() == true) {
                        telemetry.addLine("done");
                        driveOpState = AutoCode3.OperState.LIFTUP;
                        servoTimer.reset();
                    }
                    break;
                case LIFTUP:
                    ;
                    if (servoTimer.time() >= 2) {

                        driveOpState = AutoCode3.OperState.PREPMOVERIGHT;
                    }
                    break;
                case PREPMOVERIGHT:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(0, 0,moverightstrafe, 0.4, chassis.zAngle);
                    servoTimer.reset();
                    driveOpState = OperState.MOVERIGHT;
                    break;

                case MOVERIGHT:
                    telemetry.addData("front left wheel", chassis.frontLeft + " = " + (1 * chassis.drive) + " + " + (1 * chassis.strafe) + " + " + (-1 * chassis.rotation));
                    telemetry.addData("back left wheel", chassis.backLeft + " = " + (-1 * chassis.drive) + " + " + (1 * chassis.strafe) + " + " + (1 * chassis.rotation));
                    telemetry.addData("front right wheel", chassis.frontRight + " = " + (1 * chassis.drive) + " + " + (-1 * chassis.strafe) + " + " + (1 * chassis.rotation));
                    telemetry.addData("back right wheel", chassis.backRight + " = " + (1 * chassis.drive) + " + " + (1 * chassis.strafe) + " + " + (1 * chassis.rotation));
                    if (chassis.MoveToLocation() == true) {
                        driveOpState = OperState.MEASURE;
                        MultipleUsesTimer.reset();
                    }
                    break;
                    */
                case MEASURE:
                    if (MultipleUsesTimer.time(TimeUnit.SECONDS) >= 1.25) {
                        ringCount = ring.RingHeight();
                        DoneMeasuring = true;
                        driveOpState = OperState.PrepUnGetCloser;
                    }
                    break;
                case PrepUnGetCloser:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(-closermove, 0.64, 0, .4, chassis.zAngle);
                    driveOpState = OperState.UnGetCloser;
                    break;
                case UnGetCloser:
                    if (chassis.MoveToLocation()) {
                        driveOpState = OperState.UnRotate;
                    }
                    else { telemetry.addData("IMPORTANT, DRIVE PRESET", chassis.drivePreset); }
                    break;
                case UnRotate:
                    if ((Math.abs(chassis.zAngle - (originalRotation)) >= 1.5)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, (originalRotation),2));
                        chassis.Drive();
                        chassis.SetAxisMovement();
                        chassis.ZeroEncoders();
                        chassis.SetAxisMovement();
                        rotationGoal = chassis.zAngle;
                    }
                    else {
                        chassis.SetMotors(0,0,0);
                        chassis.Drive();
                        driveOpState = OperState.PrepMoveToLine;
                    }
                    //42 19 22
                    break;
                case PrepMoveToLine:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(-36, 1, 0, .4, chassis.zAngle);
                    driveOpState = OperState.MoveToLine;
                    break;
                case MoveToLine:
                    if (chassis.MoveToLocation()) { driveOpState = OperState.DECIDE; }
                    break;
                    /*
                case PREPMOVEBACK:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(0, 0, -moverightstrafe, 0.4, chassis.zAngle);
                    driveOpState = OperState.MOVEBACK;
                    break;
                case MOVEBACK:
                    telemetry.addData("front left wheel", chassis.frontLeft + " = " + (1 * chassis.drive) + " + " + (1 * chassis.strafe) + " + " + (-1 * chassis.rotation));
                    telemetry.addData("back left wheel", chassis.backLeft + " = " + (-1 * chassis.drive) + " + " + (1 * chassis.strafe) + " + " + (1 * chassis.rotation));
                    telemetry.addData("front right wheel", chassis.frontRight + " = " + (1 * chassis.drive) + " + " + (-1 * chassis.strafe) + " + " + (1 * chassis.rotation));
                    telemetry.addData("back right wheel", chassis.backRight + " = " + (1 * chassis.drive) + " + " + (1 * chassis.strafe) + " + " + (1 * chassis.rotation));
                    if (chassis.MoveToLocation() == true) {
                        servoTimer.reset();
                        driveOpState = OperState.LIFTDOWN;
                    }
                    break;

                case LIFTDOWN:

                    if (servoTimer.time() >= 2) {

                        driveOpState = AutoCode3.OperState.DECIDE;
                    }
                    break;
                    */
                case DECIDE:
                    //if ring count is 0, the values are already set to 0
                    if (ringCount == 1) {
                        targetdrive = -25;
                        targetstrafe = -22;
                        if (Powershots == 1) {
                            shootdrive = 27.5;
                            shootstrafe = -24.5;
                        }
                        else if (ShootGoals == 1) {
                            shootdrive = 27.5;
                            shootstrafe = .5;
                        }
                    }
                    else if (ringCount == 4) {
                        targetdrive = -46;
                        if (Powershots == 1) {
                            shootdrive = 48.5;
                            shootstrafe = -46.5;
                        }
                        else if (ShootGoals == 1) {
                            shootdrive = 48.5;
                            shootstrafe = -21.5;
                        }
                    }
                    else {
                        targetdrivespeed = 0;
                        if (Powershots == 1) {
                            shootdrive = 2.5;
                            shootstrafe = -46.5;
                        }
                        else if (ShootGoals == 1) {
                            shootdrive = 2.5;
                            shootstrafe = -21.5;
                        }
                    }
                    if (StartLocation == 2 || StartLocation == 3) { targetstrafe += 38; }
                    if (OnRed) {
                        targetstrafe = -targetstrafe;
                        if (Powershots == 1) {
                            shootstrafe += 19.5;
                        }
                        else {
                            shootstrafe += 7.5;
                        }
                        shootstrafe = -shootstrafe;
                    }
                    if (ShootGoals == 1) {
                        shootdrive += 8.5;
                        launchparkdrive += 8.5;
                    }
                    driveOpState = OperState.PrepGoToTargetZone;
                    break;
                case PrepGoToTargetZone:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(targetdrive, targetdrivespeed, targetstrafe, shootspeed, chassis.zAngle);
                    driveOpState = OperState.GoToTargetZone;
                    break;
                case GoToTargetZone:
                    if (chassis.MoveToLocation()) {
                        driveOpState = AutoCode3.OperState.PrepDrop;
                    }
                    break;
                case PrepDrop:
                    grabber.Open();
                    launcher.Reload(); //this is because the launcher isn't always ready by the time it gets to that point
                    GeneralTimer.reset();
                    driveOpState = OperState.Drop;
                    break;
                case Drop:
                    if (GeneralTimer.time(TimeUnit.SECONDS) >= 0.5) {
                        driveOpState = OperState.PrepMoveToShooting;
                    }
                    break;
                case PrepMoveToShooting:
                    launcher.LauncherToggle();
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(shootdrive, 1.2, shootstrafe, shootspeed, chassis.zAngle);
                    driveOpState = AutoCode3.OperState.MoveToShooting;
                    break;
                case MoveToShooting:
                    telemetry.addData("drivevalue", chassis.trueDrive);
                    telemetry.addData("strafevalue", chassis.trueStrafe);
                    telemetry.addData("drive", chassis.drive);
                    telemetry.addData("strafe", chassis.strafe);
                    if (chassis.MoveToLocation()) {
                        if (DelayAndGo == 2) { driveOpState = AutoCode3.OperState.Launch; }
                        else {
                            driveOpState = OperState.SpinAround;
                            originalRotation = chassis.zAngle;
                        }
                        servoTimer.reset();
                    }
                    break;
                case StrafeLeft:
                    if (chassis.MoveToLocation()) {
                        driveOpState = AutoCode3.OperState.Launch;
                    }
                    break;
                case SpinAround:
                    if ((Math.abs(chassis.zAngle - (originalRotation+180)) >= 3)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, (originalRotation+180),2));
                        chassis.Drive();
                        chassis.SetAxisMovement();
                        chassis.ZeroEncoders();
                        chassis.SetAxisMovement();
                        rotationGoal = chassis.zAngle;
                    }
                    else {
                        chassis.SetMotors(0,0,0);
                        chassis.Drive();
                        driveOpState = OperState.Launch;
                    }
                    break;
                case Launch:
                    if (launchCount <= 2 && (((Powershots == 1 && launchCount >= 1) && servoTimer.time() >= 0.185) || servoTimer.time() >= 1)) {
                        launcher.Shoot();
                        launchCount++;
                        driveOpState = AutoCode3.OperState.Reload;
                        MultipleUsesTimer.reset();
                    }
                    else if (launchCount > 2) { driveOpState = AutoCode3.OperState.PrepLaunchPark; }
                    break;
                case Reload:
                    if (MultipleUsesTimer.time(TimeUnit.SECONDS) >= 1 ) {
                        launcher.Reload();
                        if (Powershots == 1 && launchCount <= 2) { driveOpState = AutoCode3.OperState.PrepStrafeLeft; }
                        else { driveOpState = AutoCode3.OperState.Launch; }
                        servoTimer.reset();
                    }
                    break;
                case PrepStrafeLeft:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(0, 0, strafeslightleft, .35, chassis.zAngle);
                    driveOpState = AutoCode3.OperState.StrafeLeft; //we aren't actually moving to the goals, it's just that i don't want to have the same case being used twice. the case should really be renamed to MovingBeforeLaunch or something similar, but I really don't want to spend the time doing that right now. I probably should have really done that instead of writing this long-ass comment, but whatever. Also lmao we should probably removing the me swearing if we end up sending code to the judges for the state tournament.
                    break;
                case PrepLaunchPark:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(launchparkdrive, 1, 0, .4, chassis.zAngle);
                    driveOpState = AutoCode3.OperState.LaunchPark;
                    break;
                case LaunchPark:
                    if (chassis.MoveToLocation() == true) {
                        telemetry.addLine("done");
                    }
                    break;
                /*
                case THIRDMOVE:
                    telemetry.addLine("THIRDMOVE");
                    ;

                    if (servoTimer.time() >= 2) {

                        driveOpState = AutoCode2.OperState.STARTLAUNCHER;
                    }
                    break;

                case STARTLAUNCHER:
                    launcher.LauncherToggle();
                    driveOpState = AutoCode2.OperState.FOURTHMOVESETUP;

                case SHOOT1:
                    if (shootWait < 6) {
                        if (servoTimer.time() > 0.5) {
                            driveOpState = AutoCode2.OperState.firsttimer;
                        }
                    } else {
                        driveOpState = AutoCode2.OperState.FIFTHMOVESETUP;
                    }

                    break;
                case firsttimer:
                    servoTimer.reset();
                    driveOpState = AutoCode2.OperState.Load;
                    break;

                case Load:
                    launcher.Shoot();
                    if (servoTimer.time() >= 0.15) {
                        driveOpState = AutoCode2.OperState.secondtimer;
                    }
                    break;

                case secondtimer:
                    servoTimer.reset();
                    driveOpState = AutoCode2.OperState.ResetPosition;
                    break;

                case ResetPosition:
                    launcher.Reload();
                    if (servoTimer.time() >= 0.15) {
                        shootWait += 1;
                        servoTimer.reset();
                        driveOpState = AutoCode2.OperState.SHOOT1;
                    }
                    break;
*/
            }
            telemetry.update();
        }
    }
}
