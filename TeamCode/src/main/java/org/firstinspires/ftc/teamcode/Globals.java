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
    public static double inch;
    public static double cmToInch;
    public static CenterStageDriveBase robot;
    public static Servo Booper;
    public static double Y;
    public static double X;
    public static double wheelH;
    public static boolean SpikeIsFinished;
    public static boolean FirstMoving;
    public static double LastY;
    public static double LastX;

    public static Servo Door;
    public static Servo Pivot;
    public static Servo SLift;

    public static DcMotorEx Lift;

    public static boolean WeHaveNoGoods;
    public static boolean RESETME;




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

//        FrightDistance = FrightDetector.getDistance(DistanceUnit.MM);
//        //FrontDistance = FrontDS.getDistance(DistanceUnit.INCH);
//        LynxModule.BulkData bulkData = odoModule.getBulkData();
//
//        //cmToInch = Globals.FrontSonar.getDistanceSync();
//        //inch = cmToInch/2.54;
//        int left = bulkData.getMotorCurrentPosition(2);
//
//        //left = (int) (left * 1.02462166);
//        int right = bulkData.getMotorCurrentPosition(1);
//        int aux = bulkData.getMotorCurrentPosition(3);
//        int LiftEC = bulkData.getMotorCurrentPosition(0);
//        double POTPOS = bulkData.getAnalogInputVoltage(1);
//
//        LiftPos = LiftEC;
//
//        currentVoltage = potentiometer.getVoltage();
//
//        LiftTarget = Lift.getTargetPosition();
//        LiftPower = 0.1 + (LiftTarget - LiftPos)/LiftTarget;
//        trackingWheelIntegrator.update(left, right, aux);
//
//        Y = trackingWheelIntegrator.getY();
//        X = trackingWheelIntegrator.getX();
//        wheelH = trackingWheelIntegrator.getHeading();
//
//        opMode.telemetry.addData("X", X);
//        //opMode.telemetry.addData("FrontInch", inch);
//        opMode.telemetry.addData("Y", Y);
//        opMode.telemetry.addData("wheelH", wheelH);
//        opMode.telemetry.update();

    }
}