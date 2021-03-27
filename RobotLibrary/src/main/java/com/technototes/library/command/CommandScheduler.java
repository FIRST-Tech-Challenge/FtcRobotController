package com.technototes.library.command;

import com.technototes.library.structure.CommandOpMode;
import com.technototes.library.subsystem.Subsystem;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class CommandScheduler implements Runnable {

    private Map<Subsystem<?>, Map<Command, BooleanSupplier>> requirementCommands;
    private Map<Subsystem<?>, Command> runningRequirementCommands;
    private Map<Command, BooleanSupplier> commandsWithoutRequirements;

    public CommandOpMode opMode;
    public CommandScheduler setOpMode(CommandOpMode c){
        opMode = c;
        return this;
    }

    private static CommandScheduler instance;
    public static synchronized CommandScheduler getInstance(){
        if(instance == null){
            instance = new CommandScheduler();
        }
        return instance;
    }

    private CommandScheduler(){
        commandsWithoutRequirements = new HashMap<>();
        requirementCommands = new HashMap<>();
        runningRequirementCommands = new HashMap<>();
    }

    public CommandScheduler schedule(Command command){
        return schedule(command, ()->true);
    }
    public CommandScheduler scheduleInit(Command command, BooleanSupplier supplier){
        return scheduleForState(command, supplier, CommandOpMode.OpModeState.INIT);
    }
    public CommandScheduler scheduleJoystick(Command command, BooleanSupplier supplier){
        return scheduleForState(command, supplier, CommandOpMode.OpModeState.RUN, CommandOpMode.OpModeState.END);
    }
    public CommandScheduler scheduleForState(Command command, BooleanSupplier supplier, CommandOpMode.OpModeState... states){
        return schedule(command, ()->supplier.getAsBoolean() && opMode.getOpModeState().isState(states));
    }

    public CommandScheduler scheduleAfterOther(Command dependency, Command other){
        return schedule(other, dependency::justFinished);
    }
    public CommandScheduler scheduleWithOther(Command dependency, Command other){
        return schedule(other, dependency::justStarted);
    }
    public CommandScheduler scheduleAfterOther(Command dependency, Command other, BooleanSupplier additionalCondition){
        return schedule(other, ()->dependency.justFinished()&&additionalCondition.getAsBoolean());
    }
    public CommandScheduler scheduleWithOther(Command dependency, Command other, BooleanSupplier additionalCondition){
        return schedule(other, ()->dependency.justStarted() && additionalCondition.getAsBoolean());
    }


    public CommandScheduler schedule(Command command, BooleanSupplier supplier){
        if(command.getRequirements().isEmpty()){
            commandsWithoutRequirements.put(command, supplier);
        }else{
            command.requirements.forEach((subsystem -> {
                if(!requirementCommands.containsKey(subsystem)){
                    requirementCommands.put(subsystem, new LinkedHashMap<>());
                }
                if(subsystem.getDefaultCommand() == command){
                    runningRequirementCommands.put(subsystem, command);
                }
                requirementCommands.get(subsystem).put(command, supplier);
            }));
        }
        return this;
    }
    private Set<Command> cancelledCommands;

    @Override
    public void run() {
        cancelledCommands = new HashSet<>();
        requirementCommands.forEach(((subsystem, commandMap) -> {
            Command c = runningRequirementCommands.get(subsystem);
            commandMap.entrySet().stream().filter((entry) -> {
                        return entry.getKey().commandState == Command.CommandState.RESET
                                && entry.getValue().getAsBoolean()
                                && entry.getKey() != c;
                    }
            ).findFirst().ifPresent(m -> {
                if(c != null) {
                    cancel(c);
                    cancelledCommands.add(c);
                }
                runningRequirementCommands.put(subsystem, m.getKey());
            });
        }));
        runningRequirementCommands.forEach(((subsystem, command) -> run(command, requirementCommands.get(subsystem).get(command))));
        commandsWithoutRequirements.forEach(this::run);
        requirementCommands.keySet().forEach(Subsystem::periodic);
        cancelledCommands.forEach(Command::run);
    }
    public void run(Command command, BooleanSupplier supplier){
        if(supplier.getAsBoolean() || command.commandState != Command.CommandState.RESET){
            command.run();
        }
    }
    public void cancel(Command command){
        //force the command to end
        if(command != null) command.commandState = Command.CommandState.FINISHED;
    }

}
