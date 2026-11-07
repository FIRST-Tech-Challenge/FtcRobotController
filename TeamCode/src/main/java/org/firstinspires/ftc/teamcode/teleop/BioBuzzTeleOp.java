package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.BeamBrakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraTrackingSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.DepollenaterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
@Disabled
@Config
@TeleOp(name = "BioBuzz-broken", group = "Competition")
public class BioBuzzTeleOp extends OpMode {
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private FeederSubsystem feederSubsystem;
    private BeamBrakeSubsystem beamBrakeSubsystem;
    private CameraTrackingSubsystems cameraTrackingSubsystems;
    private DepollenaterSubsystem depollenaterSubsystem;

    private IMU imu;


    private double rx;
    private double driveSpeed = 0;
    private double driveSpeedFast = 0.95;
    private double driveSpeedSlow = 0.4;
    private double driveSpeedNormal = 0.8;
    public static double turnSpeed = 0.5;

    public static double autoAimMultuplyer = 0.05;

    private double forward;
    private double strafe;
    private double rotate;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        feederSubsystem = new FeederSubsystem(hardwareMap, telemetry);
        beamBrakeSubsystem = new BeamBrakeSubsystem(hardwareMap, telemetry);
        cameraTrackingSubsystems =new CameraTrackingSubsystems(hardwareMap,telemetry);
        depollenaterSubsystem = new DepollenaterSubsystem(hardwareMap,telemetry);
        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        beamBrakeSubsystem.Reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        shooterSubsystem.initShooter();
        depollenaterSubsystem.depollenaterUp();



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
//        drive.updatePoseEstimate();

        if (gamepad1.x) {
            imu.resetYaw();
        }

        if (gamepad1.left_bumper){
           driveSpeed = driveSpeedSlow;
        } else if (gamepad1.right_bumper){
            driveSpeed = driveSpeedFast;
        }
        else {
            driveSpeed = driveSpeedNormal;
        }

         forward = gamepad1.left_stick_y * driveSpeed; // Forward/Backward
         strafe = -gamepad1.left_stick_x * driveSpeed;  // Strafing left/right
        if (gamepad1.a){
            rotate = cameraTrackingSubsystems.autoAim() * autoAimMultuplyer * driveSpeed;// Rotation
        } else {
            rotate = -gamepad1.right_stick_x * turnSpeed;// Rotation
        }

        // Fixed: access pose through the localizer
//        Pose2d poseEstimate = drive.localizer.getPose();
//        Vector2d input = Rotation2d.exp(-poseEstimate.heading.toDouble()).times(new Vector2d(x, y));
//        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(input.y, input.x), rx));
//        drive.driveFieldRelative(y, x, rx);
        driveFieldRelative(forward, strafe, rotate);


        // endregion

        // region Intake code
        if ((gamepad2.right_trigger > 0 || gamepad2.right_bumper) & !beamBrakeSubsystem.Status()) {
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

        // region de-pollonater code
        if (gamepad2.y){
            depollenaterSubsystem.DepollenaterDown();
        } else {
            depollenaterSubsystem.depollenaterUp();
        }
        // endregion

        driveFieldRelative(driveSpeed, strafe, rotate);

    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        if (gamepad1.left_bumper) {
            maxSpeed = 0.4;
        } else if (gamepad1.right_bumper) {
            maxSpeed = 1.0;
        } else {
            maxSpeed = 0.85;
        }

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));




    }
}