package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Common;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUFactory;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndWristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@TeleOp(name = "SyborgsRefactor")
@SuppressWarnings("unused")
public class SyborgsRefactor extends LinearOpMode {
    DriveSubsystem drive;
    IMU imu;
    ArmSubsystem arm;
    LiftSubsystem lift;
    IntakeAndWristSubsystem intakeAndWrist;

    @Override
    public void runOpMode() {
        initSubsystems();
        waitForStart();
        while (opModeIsActive()) {
            drive.handleMovement(gamepad1, gamepad2, imu);

            lift.handleMovement(gamepad1, gamepad2);
            lift.updateTelemetry();

            arm.handleMovement(gamepad1, gamepad2);
            arm.updateTelemetry();
            arm.updateArmLiftCompensation(lift.getLiftPosition());

            intakeAndWrist.handleMovement(gamepad1, gamepad2);
            intakeAndWrist.updateTelemetry();


            Common.updateCycleTimes(getRuntime());
            telemetry.update();
        }
    }
    public void initSubsystems() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        imu = IMUFactory.initIMU(hardwareMap);
        arm = new ArmSubsystem(hardwareMap, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        intakeAndWrist = new IntakeAndWristSubsystem(hardwareMap, telemetry);
    }


}
