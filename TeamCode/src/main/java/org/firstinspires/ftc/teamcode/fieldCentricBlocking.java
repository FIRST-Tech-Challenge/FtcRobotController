package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.teamcode.tools.Robot;
import org.firstinspires.ftc.teamcode.tools.SetDriveMotors;
import org.firstinspires.ftc.teamcode.tools.SetDriveMotorsBlocking;

@TeleOp
public class fieldCentricBlocking extends LinearOpMode {
    private SetDriveMotorsBlocking driveMotors;
    public void Setup(){
        Global.telemetry = telemetry;
        driveMotors = new SetDriveMotorsBlocking(hardwareMap, gamepad1);


    }

    @Override
    public void runOpMode() throws InterruptedException {
        Setup();
        waitForStart();

        while (opModeIsActive()) {
            Global.telemetry.update();
            double horizontal = gamepad1.left_stick_x;
            double vertical = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            boolean goFast = gamepad1.left_bumper;

            boolean switchDriveMode = gamepad1.b;

            driveMotors.driveCommands(horizontal, vertical, turn, goFast, switchDriveMode);
        }
    }
}