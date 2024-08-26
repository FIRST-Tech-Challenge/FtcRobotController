package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.IDriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.subsystems.SympleCommandBase;

@Config
public class DriveDistanceDriveCommand extends SympleCommandBase<IDriveTrainSubsystem> {
    public static final double Kp = 0.65;
    private static final double MAX_POWER = 0.8f;

    private final PController pController;
    private final double finalPos;

    private double STARTING_POS;

    public DriveDistanceDriveCommand(IDriveTrainSubsystem driveBaseSubsystem, double meters) {
        super(driveBaseSubsystem);
        this.finalPos = meters;

        this.pController = new PController(Kp);
        this.pController.setTolerance(0.05);
    }

    @Override
    public void initialize() {
        super.initialize();
        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getDataLoggerPrefix() + "Moving " + this.finalPos + " meters");
        this.STARTING_POS = this.subsystem.getForwardDistanceDriven();
        this.pController.reset();
        this.pController.setSetPoint(this.finalPos);
    }

    @Override
    public void execute() {
        super.execute();

        double driveDistance = (this.subsystem.getForwardDistanceDriven() - this.STARTING_POS);

        double rawPower = this.pController.calculate(driveDistance);
        rawPower += Math.signum(rawPower) * DriveConstants.Ks;

        double power = Math.min(Math.max(rawPower, -MAX_POWER), MAX_POWER);


        this.getTelemetry().addData("power", power);
        this.getTelemetry().addData("dist", driveDistance);
        this.getTelemetry().addData("rel motor encoder", this.subsystem.getForwardDistanceDriven() - this.STARTING_POS);
        this.getTelemetry().addData("motor encoder", this.subsystem.getForwardDistanceDriven());
        this.getTelemetry().update();

        this.subsystem.moveSideMotors(power, power);
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.moveSideMotors(0, 0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.pController.atSetPoint();
    }
}
