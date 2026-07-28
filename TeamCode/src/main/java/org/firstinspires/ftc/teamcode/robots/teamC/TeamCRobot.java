package org.firstinspires.ftc.teamcode.robots.teamC;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.core.robot.Robot;

/** Initial Team C composition using safe shared baseline components. */
public class TeamCRobot extends Robot {
    private final RobotHardware hardware;
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final VisionSubsystem vision;
    private boolean hardwareInitialized;

    public TeamCRobot() { this(new RobotHardware()); }
    public TeamCRobot(RobotHardware hardware) {
        if (hardware == null) throw new IllegalArgumentException("Team C robot needs robot hardware.");
        this.hardware = hardware;
        drive = new DriveSubsystem(hardware.getDriveHardware());
        intake = new IntakeSubsystem(hardware.getIntakeHardware());
        vision = new VisionSubsystem(hardware.getVisionHardware());
        registerSubsystem(drive); registerSubsystem(intake); registerSubsystem(vision);
    }
    public void initialize(HardwareMap hardwareMap) {
        if (hardwareInitialized) return;
        hardware.initialize(hardwareMap); hardwareInitialized = true; super.initialize();
    }
    public void drive(double forward, double strafe, double rotate) { drive.drive(forward, strafe, rotate); }
    public void enableManualDrive() { drive.enableManualDrive(); }
    public void disableDrive() { drive.disableDrive(); }
    public void enableHeadingHold() { drive.enableHeadingHold(); }
    public void startIntake() { intake.startIntake(); }
    public void stopIntake() { intake.stopIntake(); }
    public void holdIntake() { intake.hold(); }
    public void ejectIntake() { intake.eject(); }
    public void enableVision() { vision.enableVision(); }
    public void disableVision() { vision.disableVision(); }
    public String getDriveStateName() { return drive.getCurrentStateName(); }
    public String getIntakeStateName() { return intake.getCurrentStateName(); }
    public boolean isIntakeAvailable() { return intake.isAvailable(); }
    public String getVisionStateName() { return vision.getCurrentStateName(); }
    public boolean isVisionAvailable() { return vision.isAvailable(); }
    @Override protected void onStop() { hardware.stopAll(); }

    // TODO: Replace wrappers, directions, and subsystem constants after Team C hardware is known.
    // TODO: Add Team C-specific mechanisms when their geometry is defined.
}
