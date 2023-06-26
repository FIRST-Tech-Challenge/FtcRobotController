package org.firstinspires.ftc.teamcode.commandBased.subsystems;

public class AutoSubsystems {

    private final AutoDrivetrainSubsystem drive;
    private final ElevatorSubsystem ele;
    private final ArmSubsystem arm;
    private final RotatorSubsystem rot;
    private final IntakeSubsystem intake;

    public AutoSubsystems(
            AutoDrivetrainSubsystem drive,
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot,
            IntakeSubsystem intake
    ) {
        this.drive = drive;
        this.ele = ele;
        this.arm = arm;
        this.rot = rot;
        this.intake = intake;
    }

    public AutoDrivetrainSubsystem getDrive() {
        return drive;
    }

    public ElevatorSubsystem getEle() {
        return ele;
    }

    public ArmSubsystem getArm() {
        return arm;
    }

    public RotatorSubsystem getRot() {
        return rot;
    }

    public IntakeSubsystem getIntake() {
        return intake;
    }
}
