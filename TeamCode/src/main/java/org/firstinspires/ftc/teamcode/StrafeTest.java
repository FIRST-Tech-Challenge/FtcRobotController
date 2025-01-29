package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUFactory;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndWristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.utils.MecanumMovement;

@TeleOp()
@SuppressWarnings("unused")
public class StrafeTest extends LinearOpMode {
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
            double m = -gamepad1.left_stick_y;
            double x = 0;
            double y = 0;
            if (gamepad1.dpad_up) y += m;
            if (gamepad1.dpad_down) y -= m;
            if (gamepad1.dpad_right) x += m;
            if (gamepad1.dpad_left) x -= m;
            drive.runWithCorrections(new MecanumMovement(y, x, 0), imu);

            telemetry.update();
        }
    }
    public void initSubsystems() {
        drive = new DriveSubsystem(hardwareMap, telemetry, imu);
        imu = IMUFactory.initIMU(hardwareMap);
    }
}
