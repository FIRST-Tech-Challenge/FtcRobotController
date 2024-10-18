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
        if (claw == null)
        {
            telemetry.addLine("Claw servo not found!");
            return;
        }
        if (gamepad1.a) {
            claw.setPosition(0.5);
        }
    }
    public void ClawClosed(Servo claw)
    {
        if (claw == null)
        {
            telemetry.addLine("Claw servo not found!");
            return;
        }
        if (gamepad1.b) {
            claw.setPosition(0.5);
        }
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
    public void ViperMotorCustom(DcMotor viperMotor, double lengthInches)
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

        DcMotorEx leftFront = hardwareMap.tryGet(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.tryGet(DcMotorEx.class, "leftBack");
        DcMotorEx rightFront = hardwareMap.tryGet(DcMotorEx.class, "rightFront");
        DcMotorEx rightBack = hardwareMap.tryGet(DcMotorEx.class, "rightBack");

        //TODO handle devices not found
        //For right now, just add a telemetry message but the code will still fail when it's accessed in code so gracefully handle the null case
        //This could be to exit the OpMode or to continue with the OpMode but not use the device. The latter requires checking for null in the code
        if (leftFront == null || leftBack == null || rightFront == null || rightBack == null)
        {
            telemetry.addLine("One or more motors not found!");
        } else {
            // Reverse the right side motors
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);
        }
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
        }

        //Call the function to initialize telemetry functions
        initMotorTelemetry(leftFront, "leftFront");
        initMotorTelemetry(leftBack, "leftBack");
        initMotorTelemetry(rightFront, "rightFront");
        initMotorTelemetry(rightBack, "rightBack");
        initMotorTelemetry(armMotor, "armMotor");
        initMotorTelemetry(viperMotor, "viperMotor");
        initGamepadTelemetry(gamepad1);
        initGamepadTelemetry(gamepad2);


        waitForStart();
        while(opModeIsActive()){ //while loop for when program is active
            //Code repeated during teleop goes here
            //Analogous to loop() method in OpMode
            MecanumDrive(leftFront, leftBack, rightFront, rightBack);

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

    private void MecanumDrive(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack) {
        double drive;
        double turn;
        double strafe;
        double fLeftPow, fRightPow, bLeftPow, bRightPow;
        double speedMultiplier = 0.75;


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

            if (leftFront != null) {leftFront.setPower(fLeftPow);}
            if (leftBack != null) {leftBack.setPower(bLeftPow);}
            if (rightFront != null) {rightFront.setPower(fRightPow);}
            if (rightBack != null) {rightBack.setPower(bRightPow);}
        }
    }

    //Initializes telemetry for a motor
    private void initMotorTelemetry( DcMotorEx motor, String motorName)
    {
        if (motor == null)
        {
            telemetry.addLine("Motor " + motorName + " not found!");
            return;
        }
        //Using functions for the telemetry addData method allows for the value to be updated each time the telemetry is updated
        telemetry.addLine(motorName)
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