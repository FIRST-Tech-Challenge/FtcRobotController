package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUFactory;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndWristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.utils.Common;

@TeleOp(name = "SyborgsTeleOp")
@SuppressWarnings("unused")
public class SyborgsTeleOp extends LinearOpMode {
    DriveSubsystem drive;
    IMU imu;
    ArmSubsystem arm;
    SlideSubsystem slide;
    IntakeAndWristSubsystem intakeAndWrist;
    @Override
    public void runOpMode() {

        initSubsystems();
        waitForStart();
        while (opModeIsActive()) {
            drive.handleMovementTeleOp(gamepad1, gamepad2, imu);

            slide.handleMovementTeleOp(gamepad1, gamepad2);
            slide.updateTelemetry();

            arm.handleMovementTeleOp(gamepad1, gamepad2);
            arm.updateTelemetry();
            arm.updateArmSlideCompensation(slide.getSlidePosition());

            intakeAndWrist.handleMovementTeleOp(gamepad1, gamepad2);
            intakeAndWrist.updateTelemetry();

            Common.updateCycleTimes(getRuntime());
            telemetry.update();
        }
    }
    public void initSubsystems() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        imu = IMUFactory.initIMU(hardwareMap);
        arm = new ArmSubsystem(hardwareMap, telemetry);
        slide = new SlideSubsystem(hardwareMap, telemetry);
        intakeAndWrist = new IntakeAndWristSubsystem(hardwareMap, telemetry);
    }
}