package org.firstinspires.ftc.TBlib.Subsystems;

import java.util.Map;

import com.arcrobotics.ftclib.command.Command;


public class SubsystemManager<T> {

    private Map<T, Command> commands;
    private Command currentCommand;
    private T currentKey;
    private String name;
    private boolean hasChanged = true;
    private boolean isOverride = false;

    public SubsystemManager(String name, Map<T, Command> map, T defaultCommand){
        this.name = name;
        commands = map;
        currentKey = defaultCommand;
        currentCommand = commands.get(defaultCommand);
    }

    public void run(){
        if (hasChanged) {
            currentCommand.initialize();
            hasChanged = false;
        }
        currentCommand.execute();


    }

    public void requsetState(T state){
        if (!isOverride) {
            currentCommand.end(true);
            currentCommand = commands.get(state);
            currentKey = state;
            hasChanged = true;
        }
    }

    public void overrideState(T state){
        currentCommand = commands.get(state);
        currentKey = state;
        hasChanged = true;
        isOverride = true;
    }

    public void stopOverride(){
        isOverride = false;
    }


    public boolean isFinished(){
        return currentCommand.isFinished();
    }
}
