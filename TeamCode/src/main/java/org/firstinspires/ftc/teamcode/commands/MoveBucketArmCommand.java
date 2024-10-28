package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;

import java.util.Objects;
import java.util.function.DoubleSupplier;

public class MoveBucketArmCommand extends CommandBase {

    ArmSubsystem subsystem;
    DoubleSupplier frontwardSupplier, backwardSupplier;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("MoveBucketArmCommand");

    public MoveBucketArmCommand(ArmSubsystem subsystem, DoubleSupplier frontwardSupplier,
                          DoubleSupplier backwardSupplier) {
        Objects.requireNonNull(subsystem);

        this.subsystem = subsystem;
        this.frontwardSupplier = frontwardSupplier;
        this.backwardSupplier = backwardSupplier;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {

        double power = frontwardSupplier.getAsDouble()-backwardSupplier.getAsDouble();
        dbp.debug(String.valueOf(power));

        if (power == 0) {
            subsystem.haltAllArms();
        } else {
            subsystem.manualMoveArm(power, ArmSubsystem.ARMS.BUCKET_ARM);
            // Todo: Work on this
        }
    }
}
