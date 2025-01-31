package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUFactory;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndWristSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
@SuppressWarnings("unused")
@Autonomous(name="SyborgsBetterAuto")
public class SyborgsBetterAuto extends LinearOpMode {
    DriveSubsystem drive;
    IMU imu;
    ArmSubsystem arm;
    SlideSubsystem lift;
    IntakeAndWristSubsystem intakeAndWrist;
    @Override

    public void runOpMode() {
        waitForStart();
        initSubsystems();
        sleep(3000);
        arm.setPositionAuto(arm.ARM_SCORE_SAMPLE_IN_HIGH);
        lift.handleMovementAuto(lift.SLIDE_SCORING_IN_HIGH_BASKET);
        intakeAndWrist.handleMovementAutonomous(intakeAndWrist.WRIST_FOLDED_IN, intakeAndWrist.INTAKE_DEPOSIT);
        sleep(3000);
        intakeAndWrist.handleMovementAutonomous(intakeAndWrist.WRIST_NEUTRAL, intakeAndWrist.INTAKE_OFF);
        lift.handleMovementAuto(lift.SLIDE_COLLAPSED);
        arm.setPositionAuto(arm.ARM_COLLECT);
        drive.handleMovemenetAuto(61, 0);
        drive.handleMovemenetAuto(0, 61);


    }
    public void initSubsystxems() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        imu = IMUFactory.initIMU(hardwareMap);
        arm = new ArmSubsystem(hardwareMap, telemetry);
        lift = new SlideSubsystem(hardwareMap, telemetry);
        intakeAndWrist = new IntakeAndWristSubsystem(hardwareMap, telemetry);
    }
}