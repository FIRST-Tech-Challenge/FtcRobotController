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
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Arm;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.Viper;
import org.firstinspires.ftc.teamcode.BBcode.MechanismControllers.WristClaw;

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
    private void handleGamepad1(Arm arm, Viper viper) {
        Gamepad gamepad = gamepad1;
        //Bring the arm out to hang
//        if(gamepad.right_trigger > 0 && gamepad.y) {
//            arm.MoveToHang();
//            desiredViperState = ViperState.PrepareToHang;
//            viper.ExtendHalf(0.5);
//        }
//
//        //Pull up the robot
//        if(gamepad.right_trigger > 0 && gamepad.a) {
//            desiredViperState = ViperState.PrepareToHang;
//            viper.ExtendSpecimenhang(0.5);
//        }
    }
    private void handleGamepad2(Arm arm, Viper viper, WristClaw wristClaw) {
       Gamepad gamepad = gamepad2;
        //Hang
        if (gamepad.left_trigger > 0 && gamepad.dpad_left) {
            arm.MoveToHang();
            desiredViperState = ViperState.PrepareToHang;
            viper.ExtendHalf(0.5);
        }

        //Extend to full length, limited to 18 inches at low angles
        //High Basket
        if ((gamepad.left_trigger > 0 && gamepad.dpad_up) || (desiredViperState == ViperState.Dump && arm.get_armMotor().getCurrentPosition()<150) ) {
            arm.MoveToHighBasket();
            desiredViperState = ViperState.Dump;
            viper.ExtendFull(0.75);
            wristClaw.WristDump();
        }

        //brings arm down
        if ((gamepad.left_trigger > 0 && gamepad.dpad_down) || (desiredViperState == ViperState.Closed && arm.get_armMotor().getCurrentPosition()>1300) ) {
            wristClaw.WristUp();
            desiredViperState = ViperState.Closed;
            viper.ExtendShort(1);
            arm.MoveToHome();
        }



        //Open Claw
        if(gamepad.b) {
            //Code for when B is pressed
            //telemetry.addData("Controller 1", "B");
            wristTelemetry.setValue(loopCounter);
            telemetry.update();
            wristClaw.OpenClaw();
        }

        //Close Claw
        if(gamepad.x) {
            //Code for when B is pressed
            //telemetry.addData("Controller 1", "B");
            //wristTelemetry.setValue(loopCounter);
            telemetry.update();
            wristClaw.CloseClaw();
        }

        //Move Claw Up
        if(gamepad.y) {
            wristClaw.WristUp();
        }

        //Move Claw Down
        if(gamepad.a) {
            wristClaw.WristDown();
        }

        //bring the harm up to hang specimen
//        if (gamepad2.left_bumper) {
//            arm.MoveToSpecimen();
//            desiredViperState = ViperState.PrepareToHang;
//            viper.ExtendSpecimenhang(0.5);
//        }

        //hang the specimens
//        if (gamepad2.right_bumper) {
//
//        }
    }
    @Override
    public void runOpMode() throws InterruptedException{
        // Initialization Code Goes Here
        TelemetryHelper telemetryHelper = new TelemetryHelper(this);
        //Allows for telemetry to be added to without clearing previous data. This allows setting up telemetry functions to be called in the loop or adding telemetry items within a function and not having it cleared on next loop
        telemetry.setAutoClear(false);
        //Init for the other classes this opmode pulls methods from
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
        Arm arm = new Arm(this, telemetryHelper);
        Viper viper = new Viper(this);
        WristClaw wristClaw = new WristClaw(this);
        arm.Reset();
//        wristClaw.WristInit();
//        wristClaw.CloseClaw();

        //Call the function to initialize telemetry functions
//        telemetryHelper.initMotorTelemetry( viperMotor, "viperMotor");
        telemetryHelper.initGamepadTelemetry(gamepad1);
        telemetryHelper.initGamepadTelemetry(gamepad2);
        //Where the start button is clicked, put some starting commands after
        waitForStart();
        arm.MoveToHome();

        while(opModeIsActive()){ //while loop for when program is active
            //Code repeated during teleop goes here
            //Analogous to loop() method in OpMode

            //Drive code
            drivetrain.Drive();

            handleGamepad1(arm,viper);
            handleGamepad2(arm,viper,wristClaw);
            if (arm.getIsHome())
            {
                arm.Stop();
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