package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSub;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;

public class ArmToPosition extends CommandBase {
    private final ArmSub armSub;
    private double degrees;
    boolean instantFinish;

    public ArmToPosition(ArmSub armSub, double deg, DrivetrainSub drivetrain, boolean instantFinish) {
        this.armSub = armSub;
        this.degrees = deg;
        this.instantFinish=instantFinish;
        addRequirements(armSub, drivetrain);
    }

    @Override
    public void execute() {
        armSub.setPos(armSub.convertTicks(degrees), 1);
    }

    @Override
    public boolean isFinished() {
        return  instantFinish||
                (armSub.convertDeg(armSub.armMotor.getCurrentPosition())<=degrees+2)
                &&
                (armSub.convertDeg(armSub.armMotor.getCurrentPosition())>=degrees-2);
    }

}
