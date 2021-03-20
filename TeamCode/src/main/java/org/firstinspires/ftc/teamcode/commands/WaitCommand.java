package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.technototes.library.command.Command;

public class WaitCommand extends Command {
    //TODO add to library and add clean ui
    public double sec;
    public WaitCommand(double seconds){
        sec = seconds;
    }

    @Override
    public boolean isFinished() {
        return sec > commandRuntime.seconds();
    }
}
