package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subbys.ActuatorSubby;
public class ActuatorCommand extends CommandBase {
    private ActuatorSubby subby;

    public ActuatorCommand(ActuatorSubby sub) {
        subby = sub;
        addRequirements(subby);
    }

    @Override
    public void execute() {
        if(subby.getDist() < 8.2) {
            subby.run();
        }else{
            subby.stop();
        }
    }
    public void end(boolean interrupted){
        subby.stop();
    }
}