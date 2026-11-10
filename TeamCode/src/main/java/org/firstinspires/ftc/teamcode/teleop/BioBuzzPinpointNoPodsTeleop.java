package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.archive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.BeamBrakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraTrackingSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.DepollenaterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemTeleop;

@Config
@TeleOp(name = "BioBuzz No Pods", group = "Competition")
public class BioBuzzPinpointNoPodsTeleop extends OpMode {
    private ShooterSubsystemTeleop shooterSubsystem;
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

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private GoBildaPinpointDriver pinpoint;

    private double headingOffset = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystemTeleop(hardwareMap, telemetry);
        feederSubsystem = new FeederSubsystem(hardwareMap, telemetry);
        beamBrakeSubsystem = new BeamBrakeSubsystem(hardwareMap, telemetry);
        cameraTrackingSubsystems =new CameraTrackingSubsystems(hardwareMap,telemetry);
        depollenaterSubsystem = new DepollenaterSubsystem(hardwareMap,telemetry);
        imu = hardwareMap.get(IMU.class, "imu");


        beamBrakeSubsystem.Reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        shooterSubsystem.initShooter();
        depollenaterSubsystem.depollenaterUp();

        frontRightDrive = hardwareMap.get(DcMotor.class,"FRDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class,"FLDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class,"BLDrive");
        backRightDrive = hardwareMap.get(DcMotor.class,"BRDrive");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setOffsets(0,0);

        pinpoint.resetPosAndIMU();



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

        if (gamepad1.left_bumper) {
            driveSpeed = driveSpeedSlow;
        }else if (gamepad1.right_bumper) {
            driveSpeed = driveSpeedFast;
        }else {
            driveSpeed = driveSpeedNormal;
        }

        pinpoint.update();

        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        if (gamepad1.a){
            rx = -cameraTrackingSubsystems.autoAim() * autoAimMultuplyer * driveSpeed;// Rotation
        } else {
            rx = gamepad1.right_stick_x;
        }

        double rawHeading = pinpoint.getHeading();
        double robotHeading = rawHeading - headingOffset;

        double rotX = x * Math.cos(robotHeading) - y * Math.sin(robotHeading);
        double rotY = x * Math.sin(-robotHeading) - y * Math.cos(-robotHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower * driveSpeed);
        frontRightDrive.setPower(frontRightPower * driveSpeed);
        backRightDrive.setPower(backRightPower * driveSpeed);
        backLeftDrive.setPower(backLeftPower * driveSpeed);


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

    }
}