package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode{
    public void ClawMethod(Servo claw)
    {
        if (gamepad1.a) {
            claw.setPosition(0.5);
        }
    }
    public void WristMethod(Servo wrist)
    {
        if (gamepad1.a) {
            wrist.setPosition(0.5);
        }
    }
    public void ArmMotorMethod(DcMotor armMotor)
    {
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int armTicks = 600;
        armMotor.setTargetPosition(armTicks);    //Sets Target Tick Position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
    }
    public void ViperMotorMethod(DcMotor viperMotor)
    {
        viperMotor.setDirection(DcMotor.Direction.FORWARD);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int viperTicks = 600;
        viperMotor.setTargetPosition(viperTicks);    //Sets Target Tick Position
        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperMotor.setPower(1);
    }
    @Override
    public void runOpMode() throws InterruptedException{
        // Initialization Code Goes Here
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor armMotor;
        DcMotor viperMotor;
        Servo claw;
        Servo wrist;
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        viperMotor = hardwareMap.get(DcMotor.class, "viperMotor");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

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
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            //drive inputs
            drive = gamepad1.left_stick_y * -1;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;
            if (gamepad1.right_trigger > 0) {speedMultiplier = 1;}
            if (gamepad1.left_trigger > 0) {speedMultiplier = 0.25;}

            //drive calculations
            fLeftPow = Range.clip((drive + turn + strafe) * speedMultiplier, -1, 1);
            bLeftPow = Range.clip((drive + turn - strafe) * speedMultiplier, -1, 1);
            fRightPow = Range.clip((drive - turn - strafe) * speedMultiplier, -1, 1);
            bRightPow = Range.clip((drive - turn + strafe) * speedMultiplier, -1, 1);

            frontLeft.setPower(fLeftPow);
            backLeft.setPower(bLeftPow);
            frontRight.setPower(fRightPow);
            backRight.setPower(bRightPow);
        }

    }
}