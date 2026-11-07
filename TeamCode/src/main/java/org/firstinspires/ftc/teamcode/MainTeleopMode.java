package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanisims.InTake;
import org.firstinspires.ftc.teamcode.mechanisims.LaunchSystem;
import org.firstinspires.ftc.teamcode.mechanisims.MecanumDrive;

@TeleOp
public class MainTeleopMode extends OpMode {

    double launchSpeedMultiplier = 1.0;
    double intakeSpeedMultipler = 1.0;
    LaunchSystem launchSystem;
    MecanumDrive mecanumDrive;

    InTake intake;


    @Override
    public void init() {
        launchSystem = new LaunchSystem(hardwareMap, HardwareNames.POLLEN_LAUNCH_MOTOR, HardwareNames.POLLEN_FEED_MOTOR_1, HardwareNames.POLLEN_FEED_MOTOR_2);
        mecanumDrive = new MecanumDrive(hardwareMap);
        intake = new InTake(hardwareMap);

    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        mecanumDrive.driveFieldRelative(forward, strafe, rotate);

        if (gamepad1.right_bumper) {
            launchSystem.requestLaunch();

        }
        launchSystem.update(launchSpeedMultiplier);

        intake.update(gamepad1.left_bumper ? -intakeSpeedMultipler : intakeSpeedMultipler);
    }

}
