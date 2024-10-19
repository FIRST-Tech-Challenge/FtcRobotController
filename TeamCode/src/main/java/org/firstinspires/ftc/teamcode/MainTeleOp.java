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
    double RPM312_TicksPer = 537.7;
    double viperDriveTrainTicksPerRevolution = RPM312_TicksPer;

    double targetArmDegrees = 0;
    int loopCounter = 0;
    Telemetry.Item homeFlagTelemetry = telemetry.addData("homeFlag", homeFlag);
    Telemetry.Item wristTelemetry = telemetry.addData("Wrist", "Init");


    public void ClawOpen(Servo claw)
    {
        if (claw == null)
        {
            telemetry.addLine("Claw servo not found!");
            return;
        }
            //b button
            double clawPosition = .2;
            claw.setPosition(clawPosition);
    }
    public void ClawClosed(Servo claw)
    {
        if (claw == null)
        {
            telemetry.addLine("Claw servo not found!");
            return;
        }
        double clawPosition = 0.52;
        claw.setPosition(clawPosition);
    }
    public void WristUp(Servo wrist)
    {
        if (wrist == null) {
            telemetry.addLine("Wrist servo not found!");
            return;
        }

        //y button
        wrist.setPosition(1);
    }
     public void WristFlip(Servo wrist)
     {
       if (wrist == null) {
           telemetry.addLine("Wrist servo not found!");
           return;
       }

       //y button
       wrist.setPosition(0.35);
     }
    public void WristDown(Servo wrist)
    {
        //a button
        if (wrist == null)
        {
            telemetry.addLine("Wrist servo not found!");
            return;
        }
        wrist.setPosition(.62);
    }
    public void WristCenter(Servo wrist)
    {
        //d pad up
        if (wrist == null)
        {
            telemetry.addLine("Wrist servo not found!");
            return;
        }

        wrist.setPosition(.5);
    }
    public void WristMethod(Servo wrist)
    {
        if (wrist == null)
        {
            telemetry.addLine("Wrist servo not found!");
            return;
        }
        if (gamepad1.a) {
            wrist.setPosition(0.5);
        }
    }
    public void ArmMotorRaise(DcMotorEx armMotor)
    {
        if (armMotor == null)
        {
            telemetry.addLine("Arm motor not found!");
            return;
        }
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int armTicks = 600;
        armMotor.setTargetPosition(armTicks);    //Sets Target Tick Position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.05);
        homeFlag = false;
    }
    public void ArmMotorHome(DcMotorEx armMotor)
    {
        if (armMotor == null)
        {
            telemetry.addLine("Arm motor not found!");
            return;
        }
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int armTicks = 0;
        armMotor.setTargetPosition(armTicks);    //Sets Target Tick Position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.05);
        homeFlag = true;

    }
    public void ArmMotorCustom(DcMotorEx armMotor, int degrees)
    {
        if (armMotor == null)
        {
            telemetry.addLine("Arm motor not found!");
            return;
        }

        //Total ticks for armMotor drivetrain revolution: 7125
        //90 degree rotation for armMotor drivetrain revolution: 1781.25
        //int currentPosit = armMotor.getCurrentPosition();
        //targetArmDegrees = degrees;

        double ticksPerDegree = 7125.0/360.0;
//        double currentDegrees = currentPosit/ticksPerDegree;
//        if (currentDegrees > degrees)
//        {
//            armMotor.setDirection(DcMotor.Direction.FORWARD);
//        }
//        else
//        {
//            armMotor.setDirection(DcMotor.Direction.REVERSE);
//        }
        int convert = (int) (degrees*ticksPerDegree);
        //double difference = Math.abs(convert-currentPosit);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setTargetPosition( (int) convert);    //Sets Target Tick Position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.2);
    }
    public void MoveArmToClearancePosition(DcMotorEx armMotor) {
        if (armMotor == null) {
            telemetry.addLine("Arm motor not found!");
            return;
        }
        ArmMotorCustom(armMotor, 24);
    }
    public void ViperMotorCustom(DcMotor viperMotor, double lengthInches, DcMotor armMotor)
    {
        if (viperMotor == null)
        {
            telemetry.addLine("Viper motor not found!");
            return;
        }
        //Full motor rotation = 7125 ticks
        //4 and 5/8 inches per rotation
        //~1541 ticks per inch

        //int ticksPerInch = 1541;
        double ticksPerInch = viperDriveTrainTicksPerRevolution/4.625;
        boolean minimumAngleTrue = armMotor.getCurrentPosition() < 500;
        if (minimumAngleTrue) {
            lengthInches = Math.min(lengthInches, 18);
        }
        viperMotor.setDirection(DcMotor.Direction.REVERSE);
        int extensionTicks = (int)(lengthInches*ticksPerInch);
        viperMotor.setTargetPosition(extensionTicks);    //Sets Target Tick Position
        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperMotor.setPower(0.3);

    }
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

        DcMotorEx armMotor = hardwareMap.tryGet(DcMotorEx.class, "armMotor");
        if (armMotor == null)
        {
            telemetry.addLine("Arm motor not found!");
        } else {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        DcMotorEx viperMotor = hardwareMap.tryGet(DcMotorEx.class, "viperMotor");
        if (viperMotor == null)
        {
            telemetry.addLine("Viper motor not found!");
        } else {
            viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        Servo claw = hardwareMap.tryGet(Servo.class, "claw");
        Servo wrist = hardwareMap.tryGet(Servo.class, "wrist");
        if (claw == null || wrist == null)
        {
            telemetry.addLine("Claw or wrist servos not found!");
        } else {
            WristUp(wrist);
            ClawClosed(claw);

        }

        //Call the function to initialize telemetry functions
        telemetryHelper.initMotorTelemetry( armMotor, "armMotor");
        telemetryHelper.initMotorTelemetry( viperMotor, "viperMotor");
        telemetryHelper.initGamepadTelemetry(gamepad1);
        telemetryHelper.initGamepadTelemetry(gamepad2);


        waitForStart();

        if (armMotor != null)
        {
            MoveArmToClearancePosition(armMotor);
        }

        while(opModeIsActive()){ //while loop for when program is active
            //Code repeated during teleop goes here
            //Analogous to loop() method in OpMode

            //Drive code
            drivetrain.Drive();

            //picking up
            if (gamepad1.a) {ArmMotorCustom(armMotor, 0);}
            //clearance/specimen wall grab
            if (gamepad1.x) {MoveArmToClearancePosition((armMotor));}
            //hang
            if (gamepad1.right_bumper)
            {
                //target height is 21 inches
                //(entire front claw needs to be that height and clear robot front)
                ArmMotorCustom(armMotor, 65);
            }
            if (gamepad1.dpad_up)
            {
                desiredViperState = ViperState.PrepareToHang;
                ViperMotorCustom(viperMotor, 9, armMotor);
            }
            if (gamepad1.dpad_left)
            {
                desiredViperState = ViperState.Dump;
                ViperMotorCustom(viperMotor, 27, armMotor);
            }
            if (gamepad1.dpad_down)
            {
                desiredViperState = ViperState.Closed;
                ViperMotorCustom(viperMotor, 3, armMotor);
            }
            //specimen placement
            if (gamepad1.y)
            {
                //target height: 27 inches
                //(entire front claw needs to be that height and clear robot front)
                ArmMotorCustom(armMotor, 70);
                //extension of viper slide to place specimens
            }
            //high basket
            if (gamepad1.b) {ArmMotorCustom(armMotor, 80);}

            assert armMotor != null;
            if (armMotor.getCurrentPosition() < 10  )
            {
                homeFlagTelemetry.setValue(homeFlag);
                if (homeFlag)
                {
                    armMotor.setPower(0);
                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                else {homeFlag = false;}
            }
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
                ClawOpen(claw);
            }
            if(gamepad2.left_trigger > 0)
            {
                WristFlip(wrist);
            }
            if(gamepad2.x) {
                //Code for when B is pressed
                //telemetry.addData("Controller 1", "B");
                //telemetry.update();
                //wristTelemetry.setValue(loopCounter);
                telemetry.update();
                ClawClosed(claw);
            }
            if(gamepad2.y) {
                WristUp(wrist);
            }
            if(gamepad2.a) {
                WristDown(wrist);
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