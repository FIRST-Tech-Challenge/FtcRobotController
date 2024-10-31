package org.firstinspires.ftc.teamcode.BBcode;


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

    double targetArmDegrees = 0;
    int loopCounter = 0;
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
        //Init for the other classes this opmode pulls methods from
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
        //Where the start button is clicked, put some starting commands after
        waitForStart();
        arm.MoveToClearance();

        while(opModeIsActive()){ //while loop for when program is active
            //Code repeated during teleop goes here
            //Analogous to loop() method in OpMode

            //Drive code
            drivetrain.Drive();

            //Controller A
            //picking up
           // if (gamepad1.a) {arm.MoveToHome();}

            //clearance/specimen wall grab
            //if (gamepad1.x) {arm.MoveToClearance();}

            //hang
            //if (gamepad1.right_bumper) {arm.MoveToHang();}

            //specimen placement
            //if (gamepad1.y) {arm.MoveToSpecimen();}

            //high basket
            //if (gamepad1.b) {arm.MoveToHighBasket();}

            //Extend to 9 inches
            //Low Basket
            if (gamepad2.dpad_left) {arm.MoveToSpecimen();}
            {
                desiredViperState = ViperState.PrepareToHang;
                viper.ExtendHalf();
            }

            //Extend to full length, limited to 18 inches at low angles
            //High Basket
            if (gamepad2.dpad_up) {arm.MoveToHighBasket();}
            {
                desiredViperState = ViperState.Dump;
                viper.ExtendFull();
            }

            //Extend to 3 inches, should be pressed first (add into space before opmode loop?)
            if (gamepad2.dpad_down) {arm.MoveToClearance();}
            {
                desiredViperState = ViperState.Closed;
                viper.ExtendShort();
            }

            //brings arm down to pick up samples
            if (gamepad2.dpad_right) {arm.MoveToHome();}

            if (arm.getIsHome())
            {
                arm.Stop();
            }

            //Open Claw
            if(gamepad2.b) {
                //Code for when B is pressed
                //telemetry.addData("Controller 1", "B");
                wristTelemetry.setValue(loopCounter);
                telemetry.update();
                wristClaw.OpenClaw();
            }

            //Bring the arm out to hang
            if(gamepad2.left_trigger > 0) {arm.MoveToHang();}
            {
                desiredViperState = ViperState.PrepareToHang;
                viper.ExtendHalf();
            }

            //Pull up the robot
            if(gamepad2.right_trigger > 0) {
                desiredViperState = ViperState.PrepareToHang;
                viper.ExtendSpecimenhang();
            }

            //Close Claw
            if(gamepad2.x) {
                //Code for when B is pressed
                //telemetry.addData("Controller 1", "B");
                //wristTelemetry.setValue(loopCounter);
                telemetry.update();
                wristClaw.CloseClaw();
            }

            //Move Claw Up
            if(gamepad2.y) {
                wristClaw.MoveUp();
            }

            //Move Claw Down
            if(gamepad2.a) {
                wristClaw.MoveDown();
            }

            //bring the harm up to hang specimen
            if (gamepad2.left_bumper) {arm.MoveToSpecimen();}
            {
                desiredViperState = ViperState.PrepareToHang;
                viper.ExtendSpecimenhang();
            }

            //hang the specimens
            if (gamepad2.right_bumper) {

            }

            telemetry.update();
            //Manual viper code (commented out)
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
//                if (desiredViperState == ViperState.Manual) {
//                    viperMotor.setVelocity(0);
//                    desiredViperState = ViperState.Current;
//                    int pos = viperMotor.getCurrentPosition();
//                    viperMotor.setTargetPosition(pos);
//                    viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//                  desiredViperState = ViperState.Current;
//            }
        }
    }
}