package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.features2d.ORB;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class RobotHardware {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime timeout = new ElapsedTime();

    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {

        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");

        myOpMode.telemetry.addData(">", "Initialized");
    }

    public void teleOp() {
        //Drivetrain TeleOp Code
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -myOpMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = myOpMode.gamepad1.left_stick_x;
        double yaw = myOpMode.gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double LEFT_FRONT_POWER = axial + lateral + yaw;
        double RIGHT_FRONT_POWER = axial - lateral - yaw;
        double LEFT_BACK_POWER = axial - lateral + yaw;
        double RIGHT_BACK_POWER = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(LEFT_FRONT_POWER), Math.abs(RIGHT_FRONT_POWER));
        max = Math.max(max, Math.abs(LEFT_BACK_POWER));
        max = Math.max(max, Math.abs(RIGHT_BACK_POWER));

        if (max > 1.0) {
            LEFT_FRONT_POWER /= max;
            RIGHT_FRONT_POWER /= max;
            LEFT_BACK_POWER /= max;
            RIGHT_BACK_POWER /= max;
        }

        leftFrontDrive.setPower(LEFT_FRONT_POWER*1.1);
        rightFrontDrive.setPower(RIGHT_FRONT_POWER*1.1);
        leftBackDrive.setPower(LEFT_BACK_POWER*1.1);
        rightBackDrive.setPower(RIGHT_BACK_POWER*1.1);
    }

    public void checkTelemetry(){
        myOpMode.telemetry.addData("LeftFront at", leftFrontDrive.getCurrentPosition());
        myOpMode.telemetry.addData("LeftBack at", leftBackDrive.getCurrentPosition());
        myOpMode.telemetry.addData("RightFront at", rightFrontDrive.getCurrentPosition());
        myOpMode.telemetry.addData("RightBack at", rightBackDrive.getCurrentPosition());
        myOpMode.telemetry.update();
    }
}