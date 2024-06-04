package org.firstinspires.ftc.teamcode.commands.tankDriveBase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.subsystems.TankDriveBaseSubSystem;

@Config
public class TankDriveDistanceDriveCommand extends CommandBase {
    private TankDriveBaseSubSystem driveBaseSubsystem;
    private PController pController;

    public static double Kp = 0.5;
    private static final double MAX_POWER = 0.3f;

    private double STARTING_POS;
    private final double finalPos;

    public TankDriveDistanceDriveCommand(TankDriveBaseSubSystem driveBaseSubsystem, double meters) {
        addRequirements(driveBaseSubsystem);
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.finalPos = meters;

        this.pController = new PController(Kp);
        this.pController.setTolerance(0.05);
    }

    @Override
    public void initialize() {
        super.initialize();
        this.STARTING_POS = this.driveBaseSubsystem.getLeftWheelDistanceDriven();
        this.pController.setSetPoint(this.finalPos);
    }

    @Override
    public void execute() {
        super.execute();

        double driveDistance = (this.driveBaseSubsystem.getLeftWheelDistanceDriven() - this.STARTING_POS);

        double rawPower = this.pController.calculate(driveDistance);
        rawPower += Math.signum(rawPower) * TankDriveBaseSubSystem.Ks;

        double power = Math.min(Math.max(rawPower, -MAX_POWER), MAX_POWER);


        FtcDashboard.getInstance().getTelemetry().addData("power", power);
        FtcDashboard.getInstance().getTelemetry().addData("dist", driveDistance);
        FtcDashboard.getInstance().getTelemetry().addData("rel motor encoder", this.driveBaseSubsystem.getLeftWheelDistanceDriven() - this.STARTING_POS);
        FtcDashboard.getInstance().getTelemetry().addData("motor encoder", this.driveBaseSubsystem.getLeftWheelDistanceDriven());
        FtcDashboard.getInstance().getTelemetry().update();

        this.driveBaseSubsystem.moveMotors(power, power);
    }

    @Override
    public void end(boolean interrupted) {
        this.driveBaseSubsystem.moveMotors(0, 0);
    }

    @Override
    public boolean isFinished() {
        return this.pController.atSetPoint();
    }
}
