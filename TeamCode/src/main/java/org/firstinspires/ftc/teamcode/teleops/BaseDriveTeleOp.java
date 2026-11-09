package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.GamepadConfig;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/** Shared hardware, controls, and lifecycle for drive TeleOps. */
public abstract class BaseDriveTeleOp extends CommandOpMode {

    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private GamepadConfig controls;

    @Override
    public final void initialize() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        controls = new GamepadConfig(gamepad1);
        drive.setDefaultCommand(createDriveCommand(drive, controls));
    }

    protected abstract DriveCommand createDriveCommand(Drive drive, GamepadConfig controls);

    protected double getShooterVelocity() {
        return Shooter.SHOOT_VEL;
    }


    @Override
    public final void initialize_loop() {
        updateDriveInitialization();
        telemetry.update();
        sleep(50);
    }

    protected void updateDriveInitialization() {
        telemetry.addLine("Ready for START.");
    }

    @Override
    public final void run() {
        beforeDriveRun();

        // SubsystemBase auto-registers Shooter; the scheduler runs its PID periodic once.
        super.run();

        if (controls.isShootHeld()) {
            shooter.shoot();
        } else {
            shooter.stop();
        }

        if (controls.isIntakeHeld()) {
            intake.runIntake();
        } else {
            intake.stop();
        }

        addDriveTelemetry();
        telemetry.addData("Shooter Left Power Actual", shooter.getLeftPow());
        telemetry.addData("Shooter Right Power Actual", shooter.getRightPow());
        telemetry.addData("Shooter Velocity Command", shooter.getCommandedVel());
        telemetry.addData("Shooter Left Velocity Actual", shooter.getLeftVel());
        telemetry.addData("Shooter Right Velocity Actual", shooter.getRightVel());
        telemetry.addData("Shooter Encoder Mismatch", shooter.isEncoderMismatch());
        telemetry.addData("Intake Power Command", intake.getCommandedPower());
        telemetry.addData("Intake Power Actual", intake.getPow());
        telemetry.update();
    }

    protected void beforeDriveRun() {
    }

    protected abstract void addDriveTelemetry();

    @Override
    public final void end() {
        CommandScheduler.getInstance().cancelAll();
        drive.stop();
        shooter.stop();
        intake.stop();
    }
}
