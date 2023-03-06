package org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler;

import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

import java.util.ArrayList;

public class Scheduler {
    private final RobotConfig r;
    private Scheduler(RobotConfig r){
        this.r = r;
        this.triggers = new ArrayList<>();
    }
    private static Scheduler schedulerInstance;

    private ArrayList<Trigger> triggers;

    /**
     * Should be used to initialise the Scheduler, should be called in the RobotConfig Class
     * @param r The RobotConfig instance to use in this Scheduler instance;
     * @return guaranteed non-null singleton of the Scheduler
     */
    public static Scheduler freshInstance(RobotConfig r) {
        schedulerInstance = new Scheduler(r);
        return schedulerInstance;
    }

    /**
     * To be used for registering Triggers and Subsystems automatically, knowing that the Scheduler has been initialised
     * @return unsafe singleton of the Scheduler
     */
    public static Scheduler getInstance(){
        return schedulerInstance;
    }

    public void run(){
        pollSubsystems();
        pollTriggers();
        updateSubsystems();
    }

    private void pollSubsystems(){
        for (Subsystem subsystem : r.subsystems) {
            subsystem.read();
        }
    }

    private void pollTriggers(){
        for (Trigger trigger : triggers) {
            trigger.poll();
        }
    }

    private void updateSubsystems(){
        for (Subsystem subsystem : r.subsystems) {
            subsystem.update();
        }
    }

    public void registerSubsystem(Subsystem subsystem){
        if(r.subsystems.contains(subsystem)){
            return;
        }
        r.subsystems.add(subsystem);
    }
    public void registerTrigger(Trigger trigger){
        triggers.add(trigger);
    }
}