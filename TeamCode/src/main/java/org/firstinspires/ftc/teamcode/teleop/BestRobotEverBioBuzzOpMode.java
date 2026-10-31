package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BeamBrakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraTrackingSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
@Config
@TeleOp(name = "BioBuzz", group = "Competition")
public class BestRobotEverBioBuzzOpMode extends OpMode {
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private FeederSubsystem feederSubsystem;
    private BeamBrakeSubsystem beamBrakeSubsystem;
    private CameraTrackingSubsystems cameraTrackingSubsystems;
    private MecanumDrive drive;

    private double rx;
    private double driveSpeed = 0;
    private double driveSpeedFast = 1;
    private double driveSpeedSlow = 0.4;
    private double driveSpeedNormal = 0.65;
    public static double turnSpeed = 0.5;

    public static double autoAimMultuplyer = 0.05;

    @Override
    public void init() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        feederSubsystem = new FeederSubsystem(hardwareMap, telemetry);
        beamBrakeSubsystem = new BeamBrakeSubsystem(hardwareMap, telemetry);
        cameraTrackingSubsystems =new CameraTrackingSubsystems(hardwareMap,telemetry);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        beamBrakeSubsystem.Reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        shooterSubsystem.initShooter();
    }

    @Override
    public void loop() {

        // region Beam break code
        beamBrakeSubsystem.Status();
        // endregion

        // region Shooter code
        shooterSubsystem.pidfTuning();
        shooterSubsystem.runShooter();
        // endregion

        // region Drive code
        drive.updatePoseEstimate();

        if (gamepad1.left_bumper){
           driveSpeed = driveSpeedSlow;
        } else {
            driveSpeed = driveSpeedNormal;
        }

        double  y = -gamepad1.left_stick_y * driveSpeed; // Forward/Backward
        double x = -gamepad1.left_stick_x * driveSpeed;  // Strafing left/right
        if (gamepad1.a){
             rx = cameraTrackingSubsystems.autoAim() * autoAimMultuplyer * driveSpeed;// Rotation
        } else {
             rx = -gamepad1.right_stick_x * turnSpeed;// Rotation
        }

        // Fixed: access pose through the localizer
        Pose2d poseEstimate = drive.localizer.getPose();
        Vector2d input = Rotation2d.exp(-poseEstimate.heading.toDouble()).times(new Vector2d(x, y));
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(input.y, input.x), rx));
        // endregion

        // region Intake code
        if ((gamepad2.right_trigger > 0 || gamepad2.right_bumper) /*& !beamBrakeSubsystem.Status()*/) {
            intakeSubsystem.runIntakeForward();
        } else if (gamepad2.left_trigger > 0) {
            intakeSubsystem.runIntakeReverse();
            beamBrakeSubsystem.Reset();
        } else {
            intakeSubsystem.stopIntake();
        }
        // endregion

        // region Indexer code
        if(gamepad2.left_trigger > 0){
            feederSubsystem.runIndexerReverse();
            beamBrakeSubsystem.Reset();
        } else if (gamepad2.right_bumper){
            feederSubsystem.runIndexerForward();
            beamBrakeSubsystem.Reset();
        } else {
            feederSubsystem.stopIndexer();
        }

        // endregion


    }
}