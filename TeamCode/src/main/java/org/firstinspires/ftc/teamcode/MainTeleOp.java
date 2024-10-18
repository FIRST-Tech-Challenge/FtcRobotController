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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.text.MessageFormat;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode{
    Boolean homeFlag = false;
    //Total ticks in a revolution for 117 RPM motor: 1425.1
    double RPM117_TicksPer = 1425.1;
    //Total ticks per revolution for 312 RPM motor: 537.7
    double RPM312_TicksPer = 537.7;
    double viperDriveTrainTicksPerRevolution = RPM312_TicksPer;

    double targetArmDegrees = 0;
    public void ClawOpen(Servo claw)
    {
        if (gamepad1.a) {
            claw.setPosition(0.5);
        }
    }
    public void ClawClosed(Servo claw)
    {
        if (gamepad1.b) {
            claw.setPosition(0.5);
        }
    }
    public void WristMethod(Servo wrist)
    {
        if (gamepad1.a) {
            wrist.setPosition(0.5);
        }
    }
    public void ArmMotorRaise(DcMotorEx armMotor)
    {
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
    public void ViperMotorCustom(DcMotor viperMotor, double lengthInches)
    {
        //Full motor rotation = 7125 ticks
        //4 and 5/8 inches per rotation
        //~1541 ticks per inch

        //int ticksPerInch = 1541;
        double ticksPerInch = viperDriveTrainTicksPerRevolution/4.625;
        int extensionTicks = (int)(lengthInches*ticksPerInch);

        viperMotor.setDirection(DcMotor.Direction.REVERSE);
        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperMotor.setTargetPosition(extensionTicks);    //Sets Target Tick Position
        viperMotor.setPower(0.05);
    }
    public double MinimumTicks(double viperLength)
    {   int maxExtensionLength = 34;
        double result = maxExtensionLength/(viperLength+15.5);
        double cosAngle = acos(result);
        double ticksPerDegree = 7125.0/360.0;
        return (cosAngle*ticksPerDegree);
    }
    @Override
    public void runOpMode() throws InterruptedException{
        // Initialization Code Goes Here

        //Allows for telemetry to be added to without clearing previous data. This allows setting up telemetry functions to be called in the loop or adding telemetry items within a function and not having it cleared on next loop
        telemetry.setAutoClear(false);

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotorEx armMotor;
        DcMotorEx viperMotor;
        Servo claw;
        Servo wrist;
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        viperMotor = hardwareMap.get(DcMotorEx.class, "viperMotor");
//        claw = hardwareMap.get(Servo.class, "claw");
//        wrist = hardwareMap.get(Servo.class, "wrist");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Call the function to initialize telemetry functions
        initMotorTelemetry(armMotor);
        initMotorTelemetry(viperMotor);
        initGamepadTelemetry(gamepad1);
        initGamepadTelemetry(gamepad2);


        waitForStart();
        while(opModeIsActive()){ //while loop for when program is active
            //Code repeated during teleop goes here
            //Analogous to loop() method in OpMode
            double drive;
            double turn;
            double strafe;
            double fLeftPow, fRightPow, bLeftPow, bRightPow;
            double speedMultiplier = 0.75;

            // Reverse the right side motors
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);

            //drive inputs
            drive = gamepad1.left_stick_y * -1;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;
            if (gamepad1.right_trigger > 0) {speedMultiplier = 1;}
            if (gamepad1.left_trigger > 0) {speedMultiplier = 0.25;}

            boolean isDriveEnabled = false;
            if (isDriveEnabled) {
                //drive calculations

                fLeftPow = Range.clip((drive + turn + strafe) * speedMultiplier, -1, 1);
                bLeftPow = Range.clip((drive + turn - strafe) * speedMultiplier, -1, 1);
                fRightPow = Range.clip((drive - turn - strafe) * speedMultiplier, -1, 1);
                bRightPow = Range.clip((drive - turn + strafe) * speedMultiplier, -1, 1);

                leftFront.setPower(fLeftPow);
                leftBack.setPower(bLeftPow);
                rightFront.setPower(fRightPow);
                rightBack.setPower(bRightPow);
            }
            //picking up
            if (gamepad1.a) {ArmMotorCustom(armMotor, 0);}
            //clearance/specimen wall grab
            if (gamepad1.x) {ArmMotorCustom(armMotor, 24);}
            //hang
            if (gamepad1.right_bumper)
            {
                //target height is 21 inches
                //(entire front claw needs to be that height and clear robot front)
                ArmMotorCustom(armMotor, 65);
            }
            if (gamepad1.dpad_up)
            {
                ViperMotorCustom(viperMotor, 4.625);
            }
            if (gamepad1.dpad_down)
            {
                ViperMotorCustom(viperMotor, 0);
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

//            if (armMotor.getCurrentPosition() < 15  )
//            {
//                if (homeFlag)
//                {
//                    armMotor.setPower(0);
//                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                }
//                else {homeFlag = false;}
//            }

            telemetry.update();
        }
    }
    //Initializes telemetry for a motor
    private void initMotorTelemetry( DcMotorEx motor)
    {
        //Using functions for the telemetry addData method allows for the value to be updated each time the telemetry is updated
        telemetry.addLine()
            .addData("Pos", motor::getCurrentPosition)
            .addData("Tgt", motor::getTargetPosition)
            .addData("Pwr", "%.2f", motor::getPower)
            .addData("Vel (ticks/s)", motor::getVelocity);
        telemetry.update();
    }
    //Initializes telemetry for a gamepad
    private void initGamepadTelemetry(Gamepad gamepad)
    {
        telemetry.addLine("gamepad 1")
                .addData("X", () -> {return TFAbbr(gamepad.x);})
                .addData("Y", () -> {return TFAbbr(gamepad.y);})
                .addData("A", () -> {return TFAbbr(gamepad.a);})
                .addData("B", () -> {return TFAbbr(gamepad.b);})
                .addData("RB", () -> {return TFAbbr(gamepad.right_bumper);})
                .addData("LB", () -> {return TFAbbr(gamepad.right_bumper);});
        telemetry.addLine()
                .addData("DPad Up", () -> {return TFAbbr(gamepad.dpad_up);})
                .addData("DPad Down", () -> {return TFAbbr(gamepad.dpad_down);})
                .addData("DPad Left", () -> {return TFAbbr(gamepad.dpad_left);})
                .addData("DPad Right", () -> {return TFAbbr(gamepad.dpad_right);});
        telemetry.addLine()
                .addData("LS", () -> {return MessageFormat.format("'{'{0, number, #.##} , {1, number, #.##}'}'", gamepad.left_stick_x, gamepad.left_stick_y);})
                .addData("RS", () -> {return MessageFormat.format("'{'{0, number, #.##} , {1, number, #.##}'}'", gamepad.right_stick_x, gamepad.right_stick_y);})
                .addData("LT", () -> {return gamepad.left_trigger;})
                .addData("RT", () -> {return gamepad.right_trigger;});
        telemetry.update();
    }
    //Converts boolean to T or F
    private static char TFAbbr(boolean value) {
        return value ? 'T' : 'F';
    }
}