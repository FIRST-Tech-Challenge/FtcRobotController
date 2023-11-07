package org.firstinspires.ftc.teamcode.Commands;

import java.util.ArrayList;
import java.util.List;

public class Scheduler {
    List<Command> commandList = new ArrayList<>();
    public void add(Command command){
        commandList.add(command);
        command.start();
    }
    public void update(){
        int length = commandList.size();
        List<Command> removeList = new ArrayList<>();
        for (int i = 0; i < length; i++){
            if (commandList.get(i).isFinished()){
                commandList.get(i).end();
                removeList.add(commandList.get(i));
                commandList.get(i).commandCompleted = true;
            }
            else {
                commandList.get(i).execute();
            }
        }
        for (int i = 0; i < removeList.size(); i++){
            commandList.remove(removeList.get(i));
        }
    }
}