
package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp(name="RR in tele", group="Linear OpMode")
//@Disabled
public class FieldCentricDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    protected MecanumDrive drive;
    private double  yMultiplier = 1;
    private IMU imu;
    private Pose2d startingPos = new Pose2d(0,0,Math.toRadians(90));

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() {
        // region Hardware Initialization

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        // Initialize IMU using Parameters
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        drive = new MecanumDrive(hardwareMap, startingPos);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            Pose2d poseEstimate = drive.pose;

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            YawPitchRollAngles robotOrientation;
            robotOrientation = imu.getRobotYawPitchRollAngles();
            Vector2d input = new Vector2d(-currentGamepad1.left_stick_x * yMultiplier,
                    currentGamepad1.left_stick_y * yMultiplier);//.times(Math.toRadians(poseEstimate.heading.));


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }}
