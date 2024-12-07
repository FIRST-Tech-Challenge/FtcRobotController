package org.firstinspires.ftc.teamcode.Opmode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
// 187.02 = 3
// 92.25 = 827
@Config
@TeleOp
@Disabled
public class ArmTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap, new Pose2d(0,0));
        Arm arm = new Arm(hardwareMap, drive.getArmMotor());
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Tick", arm.getCurrentArmPosition());
            telemetry.update();
        }
    }
}
