package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime; // <-- 1. Add this import

import org.firstinspires.ftc.teamcode.subsystems.CameraTrackingSubsystems;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous
public class LoadingSideShootAndParkWithCamera extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    private double drivePower;

    private ShooterSubsystem shooterSubsystem;
    private FeederSubsystem feederSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private CameraTrackingSubsystems cameraTrackingSubsystems;

    public void runOpMode() throws InterruptedException {

        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        feederSubsystem = new FeederSubsystem(hardwareMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        cameraTrackingSubsystems = new CameraTrackingSubsystems(hardwareMap, telemetry);

        frontLeft = hardwareMap.get(DcMotor.class, "FLDrive");
        frontRight = hardwareMap.get(DcMotor.class, "FRDrive");
        backLeft = hardwareMap.get(DcMotor.class, "BLDrive");
        backRight = hardwareMap.get(DcMotor.class, "BRDrive");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterSubsystem.initShooter();

        drivePower = 0.5;

        waitForStart();

        // Turn on shooter
        shooterSubsystem.runShooter();

        // Wait for shooter to spin up
        sleep(5000);

        // 2. Look for the AprilTag for up to 15 seconds
        ElapsedTime searchTimer = new ElapsedTime();
        boolean tagFound = false;

        searchTimer.reset();
        while (opModeIsActive() && searchTimer.seconds() < 15.0) {
            if (cameraTrackingSubsystems.seeApirltag()) {
                tagFound = true;
                break; // Exit loop early as soon as tag is detected
            }
            sleep(20); // Small sleep to prevent hammering the CPU/camera thread
        }

        // 3. Conditional execution based on whether the tag was seen
        if (tagFound) {
            telemetry.addData("Status", "AprilTag seen! Shooting...");
            telemetry.update();

            // Shoot
            feederSubsystem.runIndexerForward();
            intakeSubsystem.runIntakeForward();
            sleep(2000);
            feederSubsystem.stopIndexer();
            intakeSubsystem.stopIntake();
        } else {
            telemetry.addData("Status", "AprilTag timeout. Skipping shot.");
            telemetry.update();
        }

        // --- Moving / Parking Code ---
        // Drive forward
        frontLeft.setPower(drivePower);
        backLeft.setPower(drivePower);
        frontRight.setPower(drivePower);
        backRight.setPower(drivePower);
        sleep(500);
        frontLeft.setPower(0); // Stop driving
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        sleep(100);

        // Turn left
        frontLeft.setPower(-drivePower);
        backLeft.setPower(-drivePower);
        frontRight.setPower(drivePower);
        backRight.setPower(drivePower);
        sleep(310);
        frontLeft.setPower(0); // Stop driving
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        sleep(100);

        // Drive backward
        frontLeft.setPower(-drivePower);
        backLeft.setPower(-drivePower);
        frontRight.setPower(-drivePower);
        backRight.setPower(-drivePower);
        sleep(1500);
        frontLeft.setPower(0); // Stop driving
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        sleep(100);

        // Nudge forward
        frontLeft.setPower(drivePower);
        backLeft.setPower(drivePower);
        frontRight.setPower(drivePower);
        backRight.setPower(drivePower);
        sleep(100);
        frontLeft.setPower(0); // Stop driving
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        sleep(100);

        // Turn off motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        shooterSubsystem.stopShooter();
        feederSubsystem.stopIndexer();
        intakeSubsystem.stopIntake();
    }
}