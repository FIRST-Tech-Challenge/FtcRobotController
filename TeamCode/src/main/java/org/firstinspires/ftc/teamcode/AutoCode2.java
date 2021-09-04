package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.util.concurrent.TimeUnit;

//At this point it's almost entirely created by Ethan
@Autonomous
public class AutoCode2 extends LinearOpMode {

    private Blinker Control_Hub;
    private Blinker expansion_Hub_2;
    ElapsedTime servoTimer = new ElapsedTime();
    ElapsedTime GeneralTimer = new ElapsedTime();

    enum OperState {
        FIRSTMOVE,
        PREPMOVEANDLIFT,
        LIFTUP,
        DECIDE,
        LIFTDOWN,
        PREPMOVERIGHT,
        PREPMOVEBACK,
        MOVEANDLIFT,
        MOVERIGHT,
        MOVEBACK,
        RESETTIMER,
        MEASURE,
        PREPA,
        PREPB,
        PREPC,
        GoToTargetZone,
        PrepStrafeLeft,
        PrepLaunchPark,
        PrepMoveToShooting,
        MoveToPowerShots,
        StrafeLeft,
        LaunchPark,
        MoveToGoals,
        NextLocation,
        PrepSpinAround,
        SpinAround,
        Launch,
        Delayer,
        Reload
    }

    enum Menu {
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
        AutoCode2.OperState driveOpState = AutoCode2.OperState.FIRSTMOVE;
        AutoCode2.Menu menu = AutoCode2.Menu.StartLocation;
        DistanceSensorClass.RingClass ring = new DistanceSensorClass.RingClass();

        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        lift.LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
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
        double adrive = 0;
        double astrafe = 0;
        double bdrive = 0;
        double bstrafe = 0;
        double cdrive = 0;
        double cstrafe = 0;
        double shootdrive = 0;
        double shootstrafe = 0;
        double strafeslightleft = -4.75;
        int launchCount = 0;
        int ringCount = 0;
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
                        menu = AutoCode2.Menu.ButtonWaiter0;
                    } else if (gamepad1.b) {
                        StartLocation = 2;
                        OnRed = false;
                        menu = AutoCode2.Menu.ButtonWaiter0;
                    } else if (gamepad1.x) {
                        StartLocation = 3;
                        OnRed = true;
                        menu = AutoCode2.Menu.ButtonWaiter0;
                    } else if (gamepad1.y) {
                        StartLocation = 4;
                        OnRed = true;
                        menu = AutoCode2.Menu.ButtonWaiter0;
                    }
                    break;
                case ButtonWaiter0:
                    if ((StartLocation == 1 && !gamepad1.a) || (StartLocation == 2 && !gamepad1.b) || (StartLocation == 3 && !gamepad1.x) || (StartLocation == 4 && !gamepad1.y) ) { menu = AutoCode2.Menu.DelayAndGo; }
                    break;
                case DelayAndGo:
                    telemetry.addLine("Are you doing the Wobble Goals? If you aren't, then the robot will delay for 11 seconds and then go straight to the launch location. Yes (Y) No (X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        DelayAndGo = 2;
                        menu = AutoCode2.Menu.ButtonWaiter1;
                    } else if (gamepad1.y) {
                        DelayAndGo = 1;
                        menu = AutoCode2.Menu.ButtonWaiter1;
                    }
                    break;
                case ButtonWaiter1:
                    if ((DelayAndGo == 2) && (!gamepad1.x)) {
                        menu = AutoCode2.Menu.Powershots;
                    } else if ((DelayAndGo == 1) && (!gamepad1.y)) {
                        menu = AutoCode2.Menu.Powershots;
                    }
                    break;
                case Powershots:
                    telemetry.addLine("Shoot Powershots? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        Powershots = 2;
                        menu = AutoCode2.Menu.ButtonWaiter2;
                    } else if (gamepad1.y) {
                        Powershots = 1;
                        menu = AutoCode2.Menu.ButtonWaiter2;
                    }
                    break;
                case ButtonWaiter2:
                    if ((Powershots == 2 && !gamepad1.x)) {
                        menu = AutoCode2.Menu.Goals;
                    } else if ((Powershots == 1 && !gamepad1.y)) {
                        menu = AutoCode2.Menu.CheckForInvalid;
                    }
                    break;
                case Goals:
                    telemetry.addLine("Shoot into the top goal? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        ShootGoals = 2;
                        menu = AutoCode2.Menu.ButtonWaiter3;
                    } else if (gamepad1.y) {
                        ShootGoals = 1;
                        menu = AutoCode2.Menu.ButtonWaiter3;
                    }
                    break;
                case ButtonWaiter3:
                    if ((ShootGoals == 2) & (!gamepad1.x)) {
                        menu = AutoCode2.Menu.OnlyPark;
                    } else if ((ShootGoals == 1) & (!gamepad1.y)) {
                        menu = AutoCode2.Menu.CheckForInvalid;
                    }
                    break;
                case OnlyPark:
                    telemetry.addLine("Would you like to park in the corner? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        OnlyPark = 2;
                        menu = AutoCode2.Menu.ButtonWaiter4;
                    } else if (gamepad1.y) {
                        OnlyPark = 1;
                        menu = AutoCode2.Menu.ButtonWaiter4;
                    }
                    break;
                case ButtonWaiter4:
                    if ((OnlyPark == 2) & (!gamepad1.x)) {
                        menu = AutoCode2.Menu.AreYouMoving;
                    } else if ((OnlyPark == 1) & (!gamepad1.y)) {
                        menu = AutoCode2.Menu.CheckForInvalid;
                    }
                    break;
                case AreYouMoving:
                    telemetry.addLine("Would you like to stay put? Yes(Y) No(X) No will cause an error and kick you back to the beginning.");
                    telemetry.update();
                    if (gamepad1.x) {
                        AreYouMoving = 2;
                        menu = AutoCode2.Menu.ButtonWaiter5;
                    } else if (gamepad1.y) {
                        AreYouMoving = 1;
                        menu = AutoCode2.Menu.ButtonWaiter5;
                    }
                    break;
                case ButtonWaiter5:
                    if ((AreYouMoving == 2) & (!gamepad1.x)) {
                        menu = AutoCode2.Menu.CheckForInvalid;
                    } else if ((AreYouMoving == 1) & (!gamepad1.y)) {
                        menu = AutoCode2.Menu.CheckForInvalid;
                    }
                    break;
                case CheckForInvalid:
                    if (Powershots == 2 & ShootGoals == 2 & OnlyPark == 2 & AreYouMoving == 2) { menu = AutoCode2.Menu.Redo; }
                    else { menu = AutoCode2.Menu.AskIfDone; }
                    break;
                case AskIfDone:
                    telemetry.addLine("Would you like to save these changes? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        Save = 2;
                        menu = AutoCode2.Menu.ButtonWaiter6;
                    } else if (gamepad1.y) {
                        Save = 1;
                        menu = AutoCode2.Menu.ButtonWaiter6;
                    }
                    break;
                case ButtonWaiter6:
                    if ((Save == 2) & (!gamepad1.x)) {
                        menu = AutoCode2.Menu.Redo;
                    } else if ((Save == 1) & (!gamepad1.y)) {
                        menu = AutoCode2.Menu.Save;
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
                    driveOpState = AutoCode2.OperState.FIRSTMOVE;
                    menu = AutoCode2.Menu.StartLocation;
                    DoneMeasuring = false;
                    break;
                case Save:
                    if (DelayAndGo == 2) {
                        driveOpState = AutoCode2.OperState.Delayer;
                        DoneMeasuring = true;
                    }
                    if (Powershots == 1 && !OnRed) { strafeslightleft = -strafeslightleft; }
                    switch (StartLocation) {
                        case 1:
                            moverightstrafe = -6.5;
                            adrive = -28;
                            astrafe = 7;
                            bdrive = -51;
                            bstrafe = -9.5;
                            cdrive = -72;
                            cstrafe = 7;
                            if (DelayAndGo == 2) {
                                if (Powershots == 1) {
                                    shootdrive = 61.87;
                                    shootstrafe = 40;
                                }
                                
                                if (ShootGoals == 1) {
                                    shootdrive = 56.5;
                                    shootstrafe = 22.5;
                                }
                            }
                            break;

                        case 2:
                            moverightstrafe = 17;
                            adrive = -28;
                            astrafe = 30.5;
                            bdrive = -51;
                            bstrafe = 14;
                            cdrive = -72;
                            cstrafe = 30.5;
                            if (DelayAndGo == 2) {
                                if (Powershots == 1) {
                                    shootdrive = 61.87;
                                    shootstrafe = 14.5;
                                }
                                
                                if (ShootGoals == 1) {
                                    shootdrive = 56.5;
                                    shootstrafe = -4;
                                }
                            }
                            break;

                        case 3:
                            moverightstrafe = -6.5;
                            adrive = -28;
                            astrafe = -30.5;
                            bdrive = -51;
                            bstrafe = -14;
                            cdrive = -72;
                            cstrafe = -30.5;
                            if (DelayAndGo == 2) {
                                if (Powershots == 1) {
                                    shootdrive = 61.87;
                                    shootstrafe = -.5;
                                }
                                
                                if (ShootGoals == 1) {
                                    shootdrive = 56.5;
                                    shootstrafe = 22.5;
                                }
                            }
                            break;

                        case 4:
                            moverightstrafe = 17;
                            adrive = -28;
                            astrafe = -7;
                            bdrive = -51;
                            bstrafe = 9.5;
                            cdrive = -72;
                            cstrafe = -7;
                            if (DelayAndGo == 2) {
                                if (Powershots == 1) {
                                    shootdrive = 61.87;
                                    shootstrafe = -24;
                                }
                                
                                if (ShootGoals == 1) {
                                    shootdrive = 56.5;
                                    shootstrafe = -4;
                                }
                            }
                            break;
                    }
                    telemetry.addLine("Choices have been saved. You may now tell the ref you are ready.");
                    telemetry.update();
                    IsMenuDone = true;
                    break;
            }
        }

        waitForStart();
        servoTimer.reset();
        launcher.Reload();
/*                    if (chassis.MoveToLocation() == true) {
                        telemetry.addLine("done");
                    }
*/
        while (opModeIsActive()) {
            chassis.SetRotation(chassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            launcher.LauncherRun(0.939);
            //if (!DoneMeasuring) { ring.MeasureDistance(); }
            telemetry.addData("where you are in strafe", Math.abs(chassis.strafePreset - chassis.trueStrafe));
            telemetry.addData("driveopstate", driveOpState);
            telemetry.addData("IMPORTANT, DRIVE PRESET", chassis.drivePreset);
            telemetry.addData("IMPORTANT, STRAFE PRESET", chassis.strafePreset);
            telemetry.addData("Launch Count", launchCount);
            switch (driveOpState) {
                case FIRSTMOVE:
                    telemetry.addLine("FIRSTMOVE");

                    originalRotation = chassis.zAngle;

                    if (servoTimer.time() >= 2) {

                        driveOpState = AutoCode2.OperState.PREPMOVEANDLIFT;
                    }
                    break;

                case RESETTIMER:
                    MultipleUsesTimer.reset();
                    driveOpState = AutoCode2.OperState.MEASURE;
                    break;

                case Delayer:
                    if (servoTimer.time() >= 13.5) {
                        driveOpState = AutoCode2.OperState.PrepMoveToShooting;
                        servoTimer.reset();
                    }
                    break;

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
                        driveOpState = AutoCode2.OperState.LIFTUP;
                        servoTimer.reset();
                    }
                    break;
                case LIFTUP:
                    ;
                    if (servoTimer.time() >= 2) {

                        driveOpState = AutoCode2.OperState.PREPMOVERIGHT;
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
                case MEASURE:
                    if (MultipleUsesTimer.time(TimeUnit.SECONDS) >= 0.6) {
                        ringCount = ring.RingHeight();
                        driveOpState = OperState.PREPMOVEBACK;
                    }
                    break;
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

                        driveOpState = AutoCode2.OperState.DECIDE;
                    }
                    break;
                case DECIDE:
                    if (ringCount == 0) {
                        if (Powershots == 1) {
                            shootdrive = 1;
                            shootstrafe = 1;
                            if (!OnRed) {
                                shootstrafe = -shootstrafe;
                            }
                        }
                        if (ShootGoals == 1) {
                            shootdrive = 1;
                            shootstrafe = 1;
                            if (!OnRed) {
                                shootstrafe = -shootstrafe;
                            }
                        }
                        driveOpState = AutoCode2.OperState.PREPA;
                    } else if (ringCount == 1) {
                        if (Powershots == 1) {
                            shootdrive = 1;
                            shootstrafe = 1;
                            if (!OnRed) {
                                shootstrafe = -shootstrafe;
                            }
                        }
                        if (ShootGoals == 1) {
                            shootdrive = 1;
                            shootstrafe = 1;
                            if (!OnRed) {
                                shootstrafe = -shootstrafe;
                            }
                        }
                        driveOpState = AutoCode2.OperState.PREPB;
                    } else if (ringCount == 4) {
                        if (Powershots == 1) {
                            shootdrive = 1;
                            shootstrafe = 1;
                            if (!OnRed) {
                                shootstrafe = -shootstrafe;
                            }
                        }
                        if (ShootGoals == 1) {
                            shootdrive = 1;
                            shootstrafe = 1;
                            if (!OnRed) {
                                shootstrafe = -shootstrafe;
                            }
                        }
                        driveOpState = AutoCode2.OperState.PREPC;
                    }
                    DoneMeasuring = true;
                    break;
                case PREPA:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(adrive, 1, astrafe, .4, chassis.zAngle);
                    driveOpState = OperState.GoToTargetZone;
                    break;
                case PREPB:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(bdrive, 1, bstrafe, .4, chassis.zAngle);
                    driveOpState = OperState.GoToTargetZone;
                    break;
                case PREPC:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(cdrive, 1, cstrafe, .4, chassis.zAngle);
                    driveOpState = OperState.GoToTargetZone;
                    break;
                case GoToTargetZone:
                    chassis.rotationPreset -= 0.105;
                    if (chassis.MoveToLocation() == true) {
                        driveOpState = AutoCode2.OperState.PrepMoveToShooting;
                    }
                    break;

                case MoveToPowerShots:
                    if (chassis.MoveToLocation() == true) {
                        driveOpState = AutoCode2.OperState.PrepSpinAround;
                    }
                    break;
                case PrepMoveToShooting:
                    launcher.LauncherToggle();
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(shootdrive, 1.2, shootstrafe, .4, chassis.zAngle);
                    driveOpState = AutoCode2.OperState.MoveToGoals;
                    break;
                case MoveToGoals:
                    telemetry.addData("drivevalue", chassis.trueDrive);
                    telemetry.addData("strafevalue", chassis.trueStrafe);
                    telemetry.addData("drive", chassis.drive);
                    telemetry.addData("strafe", chassis.strafe);
                    if (chassis.MoveToLocation() == true) {
                        driveOpState = AutoCode2.OperState.Launch;
                        servoTimer.reset();
                    }
                    break;
                case PrepSpinAround:
                    if ((Math.abs(chassis.zAngle - (originalRotation-180)) >= 2)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, (originalRotation-180),1));
                        chassis.Drive();
                        chassis.SetAxisMovement();
                        chassis.ZeroEncoders();
                        chassis.SetAxisMovement();
                        rotationGoal = chassis.zAngle;
                    }
                    else { driveOpState = AutoCode2.OperState.Launch; }
                    break;
                case Launch:
                    if (launchCount <= 2 && (((Powershots == 1 && launchCount >= 1) && servoTimer.time() >= 0.185) || servoTimer.time() >= 1)) {
                        launcher.Shoot();
                        launchCount++;
                        driveOpState = AutoCode2.OperState.Reload;
                        MultipleUsesTimer.reset();
                    }
                    else if (launchCount > 2) { driveOpState = AutoCode2.OperState.PrepLaunchPark; }
                    break;
                case Reload:
                    if (MultipleUsesTimer.time(TimeUnit.SECONDS) >= 1 ) {
                        launcher.Reload();
                        if (Powershots == 1 && launchCount <= 2) { driveOpState = AutoCode2.OperState.PrepStrafeLeft; }
                        else { driveOpState = AutoCode2.OperState.Launch; }
                        servoTimer.reset();
                    }
                    break;
                case PrepStrafeLeft:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(0, 0, strafeslightleft, .35, chassis.zAngle);
                    driveOpState = AutoCode2.OperState.MoveToGoals; //we aren't actually moving to the goals, it's just that i don't want to have the same case being used twice. the case should really be renamed to MovingBeforeLaunch or something similar, but I really don't want to spend the time doing that right now. I probably should have really done that instead of writing this long-ass comment, but whatever. Also lmao we should probably removing the me swearing if we end up sending code to the judges for the state tournament.
                    break;
                case PrepLaunchPark:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(13, 1, 0, .4, chassis.zAngle);
                    driveOpState = AutoCode2.OperState.LaunchPark;
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
