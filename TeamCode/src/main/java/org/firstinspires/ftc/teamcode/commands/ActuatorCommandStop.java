package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subbys.ActuatorSubby;
public class ActuatorCommandStop extends CommandBase {
    private ActuatorSubby subby;

    public ActuatorCommandStop(ActuatorSubby sub) {
        subby = sub;
        addRequirements(subby);
    }

    @Override
    public void execute() {
        subby.stop();
    }
//    public void end(boolean interrupted){
//        subby.stop();
//    }
}