package org.firstinspires.ftc.teamcode.commands.tankDriveBase;

import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.subsystems.TankDriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.subsystems.SympleCommandBase;

public class TankRotateRobotByDegCommand extends SympleCommandBase<TankDriveBaseSubsystem> {
    public static final double defaultKp = 0.05f;
    private static final double MAX_POWER = 0.3f;

    private final PController pController;
    private final double degToRotate;

    private int timesDone = 0;
    private double STARTING_ANGLE;

    public TankRotateRobotByDegCommand(TankDriveBaseSubsystem driveBaseSubsystem, double degToRotate, double kp) {
        super(driveBaseSubsystem);

        this.pController = new PController(kp);
        this.pController.setTolerance(2.5);
        this.degToRotate = degToRotate;
    }

    public TankRotateRobotByDegCommand(TankDriveBaseSubsystem driveBaseSubsystem, double degToRotate) {
        this(driveBaseSubsystem, degToRotate, defaultKp);
    }

    @Override
    public void initialize() {
        super.initialize();
        this.STARTING_ANGLE = this.subsystem.getHeadingByGyro();
        this.pController.setSetPoint(Math.IEEEremainder(degToRotate + STARTING_ANGLE, 360));
        this.getDataLogger().addData(DataLogger.DataType.INFO, this.getDataLoggerPrefix() + "Rotating " + this.degToRotate + "deg");
    }

    @Override
    public void execute() {
        double headingDist = this.subsystem.getHeadingByGyro();
        double distLeft = Math.IEEEremainder(this.pController.getSetPoint() - headingDist, 360);

        double rawPower = this.pController.calculate(this.pController.getSetPoint() - distLeft);

        double power = Math.min(Math.max(rawPower, -MAX_POWER), MAX_POWER);

        this.getTelemetry().addData("power", power);
        this.getTelemetry().addData("dist left", distLeft);
        this.getTelemetry().addData("heading dist", headingDist);
        this.getTelemetry().addData("rel heading", this.subsystem.getHeadingByGyro() - this.STARTING_ANGLE);
        this.getTelemetry().addData("heading", this.subsystem.getHeadingByGyro());
        this.getTelemetry().update();

        this.subsystem.moveMotors(-power, power);
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.moveMotors(0, 0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        if (this.pController.atSetPoint()) {
            this.timesDone++;
        } else {
            this.timesDone = 0;
        }
        return this.timesDone > 5;
    }
}