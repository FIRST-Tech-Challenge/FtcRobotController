package org.firstinspires.ftc.teamcode.subsystems.actions;

public class MoveForwardCommand extends Command {
    @Override
    public void run() {

        // this should do something in a threadable way
        //

        //then it should let the original thread know it is done
        this.setChanged();
        this.notifyObservers();
    }
}
