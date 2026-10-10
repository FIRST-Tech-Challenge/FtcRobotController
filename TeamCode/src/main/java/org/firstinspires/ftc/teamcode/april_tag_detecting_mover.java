package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.navigate.snap_Mecanum_mover;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.navigate.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.drivesupport.Robot_Placement;
/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "april_tag_detecting_mover", group = "Robot")

public class april_tag_detecting_mover extends OpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    private snap_Mecanum_mover snapMecanumMover;
    private AprilTagWebcam aprilTagWebcam;

    private Robot_Placement robotPlacement;

    //defining driver profiles
    private static final int RED_1 = 1;
    private static final int RED_2 = 2;
    private static final int BLUE_1 = 3;
    private static final int BLUE_2 = 4;

    public void snap_to_tag() {
        if (aprilTagWebcam.percent_value < 100) {
            if (aprilTagWebcam.x_value < 0) {
                snapMecanumMover.driveFieldRelative(0.0, -0.2, 0.0);
            } else if (aprilTagWebcam.x_value > 0) {
                snapMecanumMover.driveFieldRelative(0.0, 0.2, 0.0);
            } else {
                snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0);
            }
            //aprilTagWebcam.displayDetections(telemetry);
        }
    }

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        snapMecanumMover = new snap_Mecanum_mover(
                frontLeftDrive,
                frontRightDrive,
                backLeftDrive,
                backRightDrive,
                imu
        );
        robotPlacement = new Robot_Placement();
        robotPlacement.init(RED_1);
        aprilTagWebcam = new AprilTagWebcam();
        aprilTagWebcam.init(hardwareMap, telemetry, robotPlacement.target_tag);

    }

    @Override
    public void loop() {


        boolean tagDetected = aprilTagWebcam.updateDetections();
        boolean driver_snapping = gamepad1.right_bumper && tagDetected;
        snapMecanumMover.loop(gamepad1, telemetry, driver_snapping);
        if (driver_snapping) {
            snap_to_tag();
        }
    }
}


