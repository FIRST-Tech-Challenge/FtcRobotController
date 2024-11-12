package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.*;

@TeleOp (name="PID Test", group = "TeleOp")
//@Disabled
public class PIDTest extends LinearOpMode {
    DcMotorEx motor;
    double currentVelocity = 0.0;
    double maxVelocity = 0.0;
    double resultMaxVelocity = 2800.0;
    double minVelocity = 99999.0;
    double targetVelocity = resultMaxVelocity * .8;
    double F = 32767.0/resultMaxVelocity;
    // first tune P, then D and then I
    double kP = 1.2;//F * 0.1; //P term oscillation - bring to target velocity as soon as possible without overshoot
    double kI = kP * 0.1; // increase to give nudge you to your target over time
    // want to get to targetVelocity as soon as possible
    double kD = kP * 0.01; //applies to braking force to control overshoot, keep this small to avoid noise - keep as low as possible
    double position = 5.0; // default is 5

    @Override
    public void runOpMode() {
//        telemetry = FtcDashboard.getTelemetry();

        telemetry.addData("F", F);
        Log.i("BAB", "F: "+ F);
        telemetry.addData("kP", kP);
        Log.i("BAB", "kP: "+ kP);
        telemetry.addData("kI", kI);
        Log.i("BAB", "kI: "+ kI);
        telemetry.addData("kD", kD);
        Log.i("BAB", "kD: "+ kD);



        telemetry.update();

        motor = hardwareMap.get(DcMotorEx.class, "motor_fr");
//        initMotor(motor);
        initMotor(motor, kP, kI, kD, F, position);
        waitForStart();
        while (opModeIsActive()) {
////            motor.setPower(1);
//            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//////            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            // Set the motor's target position to 300 ticks
//            motor.setTargetPosition(300);
//
//            // Switch to RUN_TO_POSITION mode
//            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setVelocity(targetVelocity);
            currentVelocity = motor.getVelocity();
            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            } else if (currentVelocity < minVelocity) {
                minVelocity = currentVelocity;
            }
            telemetry.addData("target velocity", targetVelocity);
            Log.i("BAB", "target velocity : "+ targetVelocity);
            telemetry.addData("current power", motor.getPower());
            Log.i("BAB", "current power : "+ motor.getPower());
            telemetry.addData("current velocity", motor.getVelocity());
            Log.i("BAB", "current velocity : "+ currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            Log.i("BAB", "maximum velocity : "+ maxVelocity);
            telemetry.addData("Minimum velocity", minVelocity);
            Log.i("BAB", "minimum velocity : "+ minVelocity);
            telemetry.update();
        }

//        motor = hardwareMap.get(DcMotorEx.class, "motor_bl");
//        initMotor(motor, kP, kI, kD, F, position);
//        telemetry.addData("F", F);
//        Log.i("BAB", "F: "+ F);
//        telemetry.addData("kP", kP);
//        Log.i("BAB", "kP: "+ kP);
//        telemetry.addData("kI", kI);
//        Log.i("BAB", "kI: "+ kI);
//        telemetry.addData("kD", kD);
//        Log.i("BAB", "kD: "+ kD);
//
//        telemetry.update();
//        waitForStart();
//        while (opModeIsActive()) {
//
////            runMotor(motor, targetVelocity);
////            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
////            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
////            // Set the motor's target position to 300 ticks
////            motor.setTargetPosition(3000);
////
////            // Switch to RUN_TO_POSITION mode
////            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
////            motor.setVelocity(targetVelocity);
//            motor.setPower(1.0);
//            currentVelocity = motor.getVelocity();
//            if (currentVelocity > maxVelocity) {
//                maxVelocity = currentVelocity;
//            }
//            telemetry.addData("target velocity", targetVelocity);
//            Log.i("BAB", "target velocity : "+ targetVelocity);
//            telemetry.addData("current power", motor.getPower());
//            Log.i("BAB", "current power : "+ motor.getPower());
//            telemetry.addData("current velocity", currentVelocity);
//            Log.i("BAB", "current velocity : "+ currentVelocity);
//            telemetry.addData("maximum velocity", maxVelocity);
//            Log.i("BAB", "maximum velocity : "+ maxVelocity);
//            telemetry.update();
//
//
////            motor.setPower(1);
////            currentVelocity = motor.getVelocity();
////            if (currentVelocity > maxVelocity) {
////                maxVelocity = currentVelocity;
////            } else if (currentVelocity < minVelocity) {
////                minVelocity = currentVelocity;
////            }
////            telemetry.addData("current power", motor.getPower());
////            Log.i("BAB", "current power : "+ motor.getPower());
////            telemetry.addData("current velocity", currentVelocity);
////            Log.i("BAB", "current velocity : "+ currentVelocity);
////            telemetry.addData("maximum velocity", maxVelocity);
////            Log.i("BAB", "maximum velocity : "+ maxVelocity);
////            telemetry.addData("Minimum velocity", minVelocity);
////            Log.i("BAB", "minimum velocity : "+ minVelocity);
////            telemetry.update();
//        }
    }

    public void runMotor(DcMotorEx driveMotor, double velocity) {
        // Set the motor's target position to 300 ticks
        driveMotor.setTargetPosition(300);

        // Switch to RUN_TO_POSITION mode
        driveMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        driveMotor.setVelocity(velocity);
        driveMotor.setPower(0.8);
        currentVelocity = driveMotor.getVelocity();
        if (currentVelocity > maxVelocity) {
            maxVelocity = currentVelocity;
        }
        telemetry.addData("target velocity", targetVelocity);
        Log.i("BAB", "target velocity : "+ targetVelocity);
        telemetry.addData("current power", driveMotor.getPower());
        Log.i("BAB", "current power : "+ driveMotor.getPower());
        telemetry.addData("current velocity", currentVelocity);
        Log.i("BAB", "current velocity : "+ currentVelocity);
        telemetry.addData("maximum velocity", maxVelocity);
        Log.i("BAB", "maximum velocity : "+ maxVelocity);
        telemetry.update();
    }
    public void initMotor(DcMotorEx driveMotor, double kP, double kI, double kD, double F, double position){
        driveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        driveMotor.setPower(0.0);
        driveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        driveMotor.setVelocityPIDFCoefficients(kP, kI, kD, F);
        driveMotor.setPositionPIDFCoefficients(position);
        driveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//
//        driveMotor.setDirection(DcMotorEx.Direction.FORWARD);
//        driveMotor.setPower(0.0);
//        driveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        driveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        driveMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}