package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Commands.Command;

public class Wait extends Command {
        long time;
    long endTime;
    public Wait(long time){
                this.time = time;
    }
    public void start(){
        endTime = System.currentTimeMillis() + time;
    }
    public void execute(){

    }
    public void end(){
    }

    public boolean isFinished() {
        if (System.currentTimeMillis() >= endTime){
            return true;
        }
        else {
            return false;
        }
    }
}
