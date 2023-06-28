package org.firstinspires.ftc.teamcode.commandBased.subsystems;

public class Subsystems {

    private final AutoDrivetrainSubsystem rrDrive;
    private final ElevatorSubsystem ele;
    private final ArmSubsystem arm;
    private final RotatorSubsystem rot;
    private final IntakeSubsystem intake;

    public Subsystems(
            AutoDrivetrainSubsystem drive,
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot,
            IntakeSubsystem intake
    ) {
        this.rrDrive = drive;
        this.ele = ele;
        this.arm = arm;
        this.rot = rot;
        this.intake = intake;
    }

    public AutoDrivetrainSubsystem rrDrive() {
        return rrDrive;
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
