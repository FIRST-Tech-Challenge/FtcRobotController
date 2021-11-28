package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DriveMotor;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.kinematics.MecanumKinematics;
import org.firstinspires.ftc.teamcode.utils.Pose2D;

import static org.firstinspires.ftc.teamcode.core.ROBOT_DATA.*;

@TeleOp(name="Drive Test", group="Tests")
public class DriveTest extends OpMode
{
    MecanumKinematics kinematics;
    MecanumDrive drive;

    DriveMotor leftFrontDrive, rightFrontDrive, leftRearDrive, rightRearDrive;

    @Override
    public void init() {
        kinematics = MecanumKinematics.getInstance();

        leftFrontDrive = new DriveMotor(hardwareMap, LEFT_FRONT_DRIVE_ID, LEFT_FRONT_DIRECTION);
        rightFrontDrive = new DriveMotor(hardwareMap, RIGHT_FRONT_DRIVE_ID, RIGHT_FRONT_DIRECTION);
        leftRearDrive = new DriveMotor(hardwareMap, LEFT_REAR_DRIVE_ID, LEFT_REAR_DIRECTION);
        rightRearDrive = new DriveMotor(hardwareMap, RIGHT_REAR_DRIVE_ID, RIGHT_REAR_DIRECTION);

        drive = new MecanumDrive(hardwareMap, new Pose2D(), leftFrontDrive, rightFrontDrive, leftRearDrive, rightRearDrive);
    }

    @Override
    public void loop() {
        // Drive!!!
        drive.driveWithGamepad(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
