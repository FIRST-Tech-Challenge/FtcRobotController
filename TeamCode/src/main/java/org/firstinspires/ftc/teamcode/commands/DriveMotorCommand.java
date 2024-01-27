package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subbys.ActuatorSubby;
import org.firstinspires.ftc.teamcode.subbys.DriveMotorSubsystem;

public class DriveMotorCommand extends CommandBase {
    private DriveMotorSubsystem subby;
    double fLV, fRV, bLV, bRV, dist;

    public DriveMotorCommand(DriveMotorSubsystem sub, double fLVE, double fRVE, double bLVE, double bRVE, double distance) {
        fLV = fLVE;
        fRV = fRVE;
        bLV = bLVE;
        bRV = bRVE;

        dist = distance;
        subby = sub;
        addRequirements(subby);
    }

    @Override
    public void execute() {
        if(subby.getDist() < dist) {
            subby.setMotorSpeed(fLV, fRV, bLV, bRV);
        }else{
            subby.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted){
        subby.stop();
    }
}