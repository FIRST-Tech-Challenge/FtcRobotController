package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.rustlib.control.PIDController;
import org.rustlib.drive.DriveSubsystem;
import org.rustlib.drive.MecanumBase;
import org.rustlib.drive.Odometry;
import org.rustlib.geometry.Rotation2d;
import org.rustlib.hardware.PairedEncoder;
import org.rustlib.rustboard.Rustboard;

public class Drive extends DriveSubsystem {
    private final MecanumBase base;
    private final Odometry odometry;
    private Rotation2d fieldCentricOffset = new Rotation2d();
    private double multiplier = Mode.FAST.multiplier;

    public Drive(HardwareMap hardwareMap) {
        odometry = Odometry.getBuilder()
                .defineLeftEncoder(new PairedEncoder(hardwareMap.get(DcMotor.class, "rb"), true))
                .defineRightEncoder(new PairedEncoder(hardwareMap.get(DcMotor.class, "lb"), true))
                .defineBackEncoder(new PairedEncoder(hardwareMap.get(DcMotor.class, "climbMotor")))
                .setTrackWidth(DriveConstants.Odometry.trackWidth)
                .setVerticalDistance(DriveConstants.Odometry.verticalDistance)
                .setInPerTick(DriveConstants.Odometry.inPerTick)
                .build();
        base = MecanumBase.getBuilder()
                .defineLeftFront(hardwareMap.get(DcMotor.class, "lf"), true)
                .defineRightFront(hardwareMap.get(DcMotor.class, "rf"))
                .defineLeftBack(hardwareMap.get(DcMotor.class, "lb"), true)
                .defineRightBack(hardwareMap.get(DcMotor.class, "rb"))
                .setPoseSupplier(odometry::getPose)
                .setMaxEndpointErr(DriveConstants.maxEndpointErr)
                .setUseEndpointHeadingDistance(DriveConstants.trackEndpointHeadingMaxDistance)
                .setTargetHeadingCalculationDistance(DriveConstants.calculateTargetHeadingMinDistance)
                .setMaxFinalVelocity(DriveConstants.maxFinalVelocityInPerSec)
                .setDriveGains(DriveConstants.driveGains)
                .setRotGains(DriveConstants.rotGains)
                .build();
    }

    @Override
    public Odometry getOdometry() {
        return odometry;
    }

    @Override
    public MecanumBase getBase() {
        return base;
    }

    public void drive(double drive, double strafe, double turn, double heading) {
        base.drive(drive * multiplier, strafe * multiplier, turn * multiplier, heading - fieldCentricOffset.getAngleRadians(), false);
        Rustboard.updateTelemetryNode("input", "drive: " + drive + " strafe: " + strafe + " turn: " + turn);
    }

    @Override
    public void drive(double drive, double strafe, double turn) {
        drive(drive, strafe, turn, odometry.getPose().rotation.getAngleRadians());
    }

    public void setFieldCentricOffset(Rotation2d fieldCentricOffset) {
        this.fieldCentricOffset = fieldCentricOffset;
    }

    public void enableFastMode() {
        multiplier = Mode.FAST.multiplier;
    }

    public void enableSlowMode() {
        multiplier = Mode.SLOW.multiplier;
    }

    @Override
    public void periodic() {
        Rustboard.updateTelemetryNode("pose", odometry.getPose().toString());
        base.driveController.setGains(new PIDController.PIDGains(
                Rustboard.getDouble("drive kP", 0.1),
                Rustboard.getDouble("drive kI", 0.0),
                Rustboard.getDouble("drive kD", 0.0001)));
        base.rotController.setGains(new PIDController.PIDGains(
                Rustboard.getDouble("rot kP", 1.0),
                Rustboard.getDouble("rot kI", 0.0),
                Rustboard.getDouble("rot kD", 0.0)));
    }

    public enum Mode {
        FAST(1.0),
        SLOW(0.25);

        final double multiplier;

        Mode(double multiplier) {
            this.multiplier = multiplier;
        }
    }
}
