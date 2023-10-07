//  _____                _           _       _  _    __  _____ _  _
// |  ___| __ ___   __ _| |__   ___ | |_ ___| || |  / /_|___ /| || |
// | |_ | '__/ _ \ / _` | '_ \ / _ \| __/ __| || |_| '_ \ |_ \| || |_
// |  _|| | | (_) | (_| | |_) | (_) | |_\__ \__   _| (_) |__) |__   _|
// |_|  |_|  \___/ \__, |_.__/ \___/ \__|___/  |_|  \___/____/   |_|
//                 |___/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.FtcAboutActivity;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivebase.CenterStageDriveBase;

public class Globals {

    public static CenterStageDriveBase driveBase;
    public static TrackingWheelIntegrator trackingWheelIntegrator;
    public static LinearOpMode opMode;
    public static LynxModule odoModule;
    public static TouchSensor LeftTouch;
    public static TouchSensor RightTouch;
    public static TouchSensor BackTouch;
    public static double inch;
    public static double cmToInch;
    public static CenterStageDriveBase robot;
    public static double LiftLevel;
    public static DcMotorEx Lift;
    public static Servo Booper;
    public static int LiftPos;
    public static Servo TSELift;
    public static Servo TSERotation;
    public static Servo TSEClaw;
    public static Servo RotationI;
    public static Servo HEXCLAW;
    public static Servo ARML;
    public static Servo ARMR;
    //public static CRServo DUCKwheel;
    public static DcMotor Intake;
    public static DcMotorEx Turret;
    public static CRServo DUCKwheel;
    public static DistanceSensor FrightDetector;
    //public static DistanceSensor FrontDS;
    public static double FrightDistance;
    public static double LiftPower;
    public static double LiftTarget;
    public static DcMotorEx FL;
    public static DcMotorEx FR;
    public static DcMotorEx RR;
    public static DcMotorEx RL;
    public static DcMotor TSEMotor;
    public static double Y;
    public static double X;
    public static double wheelH;
    public static TouchSensor LiftLimit;
    public static DcMotorEx leftTW;
    public static DcMotorEx rightTW;
    public static DcMotorEx backTW;
    public static AnalogInput potentiometer;
    public static double currentVoltage;
    public static boolean TRANSFERGoing;
    public static boolean FIRSTMOVEMENT;
    public static boolean LiftIsDown;
    public double DSStartPos = 20;
    public static boolean TurretTurn;
    public static double TurretTARGET;
    public static double TurretPos;
    public static double turnPower;
    public static double FrontDistance;
    public static boolean LOWERARM;
    public static double TSEPOWER;
    public static boolean FirstMoving;
    public static boolean MineralInClaw;
    public static boolean WeHaveTheGoods = false;
    public static double Cycle;
    public static double ResetCycle;
    public static double LastY;
    public static double LastX;
    public static boolean ReadyToPlace;
    public static boolean GoDucks;
    public static boolean NoDuckPark;
    static LynxDcMotorController ctrl;
    public static CRServo ATW;
    public static CRServo LTW;
    public static CRServo RTW;

//    public static void clearEnc()        {
//        ctrl.setMotorMode(3, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        ctrl.setMotorMode(1, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        ctrl.setMotorMode(2, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        ctrl.setMotorMode(3, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ctrl.setMotorMode(1, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ctrl.setMotorMode(2, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//    }

    public static void ResetHX() {
        LastX =trackingWheelIntegrator.getX();
        LastY =trackingWheelIntegrator.getY();

        //offset = -trackingWheelIntegrator.getHeading();
        //trackingWheelIntegrator.setHeadingOffsetDegrees(offset);

    }

    public static void updateTracking()
    {

        FrightDistance = FrightDetector.getDistance(DistanceUnit.MM);
        //FrontDistance = FrontDS.getDistance(DistanceUnit.INCH);
        LynxModule.BulkData bulkData = odoModule.getBulkData();

        //cmToInch = Globals.FrontSonar.getDistanceSync();
        //inch = cmToInch/2.54;
        int left = bulkData.getMotorCurrentPosition(2);

        //left = (int) (left * 1.02462166);
        int right = bulkData.getMotorCurrentPosition(1);
        int aux = bulkData.getMotorCurrentPosition(3);
        int LiftEC = bulkData.getMotorCurrentPosition(0);
        double POTPOS = bulkData.getAnalogInputVoltage(1);

        LiftPos = LiftEC;

        currentVoltage = potentiometer.getVoltage();

        LiftTarget = Lift.getTargetPosition();
        LiftPower = 0.1 + (LiftTarget - LiftPos)/LiftTarget;
        trackingWheelIntegrator.update(left, right, aux);

        Y = trackingWheelIntegrator.getY();
        X = trackingWheelIntegrator.getX();
        wheelH = trackingWheelIntegrator.getHeading();

        opMode.telemetry.addData("X", X);
        //opMode.telemetry.addData("FrontInch", inch);
        opMode.telemetry.addData("Y", Y);
        opMode.telemetry.addData("wheelH", wheelH);
        opMode.telemetry.addData("MineralDetection", FrightDistance );
        //opMode.telemetry.addData("Front Distance", FrontDistance );
        opMode.telemetry.addData("LiftEC", LiftEC);
        opMode.telemetry.addData("LiftPower", LiftPower);
        opMode.telemetry.addData("LiftPower", LiftTarget);
        opMode.telemetry.addData("Level", LiftLevel);
        opMode.telemetry.addData("Potentiometer voltage", currentVoltage);
        opMode.telemetry.addData("Turret Turn Power; ", turnPower);
        //opMode.telemetry.addData("TSE Power; ", TSEPOWER);
        //opMode.telemetry.addData("LiftLimit", LiftLimit.isPressed());
        opMode.telemetry.update();

    }
}