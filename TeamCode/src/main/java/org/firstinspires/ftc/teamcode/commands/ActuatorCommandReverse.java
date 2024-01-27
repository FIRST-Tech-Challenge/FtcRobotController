package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subbys.ActuatorSubby;
public class ActuatorCommandReverse extends CommandBase {
    private ActuatorSubby subby;

    public ActuatorCommandReverse(ActuatorSubby sub) {
        subby = sub;
        addRequirements(subby);
    }

    @Override
    public void execute() {
        if(subby.getDist() > 1.8    ) {
            subby.reverse();
        }else{
            subby.stop();
        }
    }
    public void end(boolean interrupted){
        subby.stop();
    }
}