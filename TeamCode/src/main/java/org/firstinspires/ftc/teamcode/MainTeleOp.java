package org.firstinspires.ftc.teamcode;


//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static java.lang.Math.acos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode{
    enum ViperState {
        Current,
        Closed,
        Manual,
        PrepareToHang,
        Dump
    }
    ViperState desiredViperState = ViperState.Closed;
    Boolean homeFlag = false;

    //Total ticks in a revolution for 117 RPM motor: 1425.1
    double RPM117_TicksPer = 1425.1;
    //Total ticks per revolution for 312 RPM motor: 537.7

    double targetArmDegrees = 0;
    int loopCounter = 0;
    Telemetry.Item homeFlagTelemetry = telemetry.addData("homeFlag", homeFlag);
    Telemetry.Item wristTelemetry = telemetry.addData("Wrist", "Init");

    public double MinimumTicks(double viperLength)
    {   int maxExtensionLength = 34;
        double result = maxExtensionLength/(viperLength+15.5);
        double cosAngle = acos(result);
        double ticksPerDegree = 538.0/360.0;
        return (cosAngle*ticksPerDegree);
    }
    @Override
    public void runOpMode() throws InterruptedException{
        // Initialization Code Goes Here
        TelemetryHelper telemetryHelper = new TelemetryHelper(this);
        //Allows for telemetry to be added to without clearing previous data. This allows setting up telemetry functions to be called in the loop or adding telemetry items within a function and not having it cleared on next loop
        telemetry.setAutoClear(false);

        MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
        Arm arm = new Arm(this);
        Viper viper = new Viper(this);
        WristClaw wristClaw = new WristClaw(this);
        arm.Reset();
        wristClaw.MoveUp();
        wristClaw.CloseClaw();

        //Call the function to initialize telemetry functions
//        telemetryHelper.initMotorTelemetry( viperMotor, "viperMotor");
        telemetryHelper.initGamepadTelemetry(gamepad1);
        telemetryHelper.initGamepadTelemetry(gamepad2);

        waitForStart();

        arm.MoveToClearance();

        while(opModeIsActive()){ //while loop for when program is active
            //Code repeated during teleop goes here
            //Analogous to loop() method in OpMode

            //Drive code
            drivetrain.Drive();

            //picking up
            if (gamepad1.a) {arm.MoveToHome();}

            //clearance/specimen wall grab
            if (gamepad1.x) {arm.MoveToClearance();}

            //hang
            if (gamepad1.right_bumper)
            {
                arm.MoveToHang();
            }
            //Extend to 9 inches
            if (gamepad1.dpad_up)
            {
                desiredViperState = ViperState.PrepareToHang;
                viper.ExtendHalf();
            }
            if (gamepad1.dpad_left)
            {
                desiredViperState = ViperState.Dump;
                viper.ExtendFull();
            }
            if (gamepad1.dpad_down)
            {
                desiredViperState = ViperState.Closed;
                viper.ExtendShort();
            }

            //specimen placement
            if (gamepad1.y)
            {
                arm.MoveToSpecimen();
            }

            //high basket
            if (gamepad1.b) {arm.MoveToHighBasket();}

            if (arm.getIsHome())
            {
                arm.Stop();
            }
//            if (armMotor.getCurrentPosition() < 10  )
//            {
//                homeFlagTelemetry.setValue(homeFlag);
//                if (homeFlag)
//                {
//                    armMotor.setPower(0);
//
//                }
//                else {homeFlag = false;}
        //}
//            if (desiredViperState == ViperState.Closed && viperMotor.getCurrentPosition() < 10)
//            {
//                viperMotor.setPower(0);
//                viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }


            //Controller B
            if(gamepad2.b) {
                //Code for when B is pressed
                //telemetry.addData("Controller 1", "B");
                wristTelemetry.setValue(loopCounter);
                telemetry.update();
                wristClaw.OpenClaw();
            }
            if(gamepad2.left_trigger > 0)
            {
                wristClaw.MoveFlip();
            }
            if(gamepad2.x) {
                //Code for when B is pressed
                //telemetry.addData("Controller 1", "B");
                //telemetry.update();
                //wristTelemetry.setValue(loopCounter);
                telemetry.update();
                wristClaw.CloseClaw();
            }
            if(gamepad2.y) {
                wristClaw.MoveUp();
            }
            if(gamepad2.a) {
                wristClaw.MoveDown();
            }
//            if (Math.abs((gamepad2.right_stick_y)) > 0.2 )
//            {
//                double motorPower = 0;
//                if (viperMotor.getCurrentPosition() > -2000)
//                {
//                    motorPower = -0.2;
//                    desiredViperState = ViperState.Manual;
//                    //538 ticks per revolution
//                    viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                }
//                else {
//                    motorPower = 0;
//                }
//                viperMotor.setPower(gamepad2.right_stick_y * motorPower);
//            } else
//            {
////                if (desiredViperState == ViperState.Manual) {
////                    viperMotor.setVelocity(0);
////                    desiredViperState = ViperState.Current;
////                    int pos = viperMotor.getCurrentPosition();
////                    viperMotor.setTargetPosition(pos);
////                    viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                }
//                  desiredViperState = ViperState.Current;
//            }
            telemetry.update();
        }
    }
}