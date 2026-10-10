package org.firstinspires.ftc.teamcode.robot.actions;

import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.SWEEPAction;

import org.firstinspires.ftc.teamcode.robot.BioBuzzRobot;

public class ExampleAction extends SWEEPAction {
    /**
     * Creates an example action that triggers around a pos2D and can run for a set amount of time.
     * Used while running SWEEP Sequences
     *
     * @param pos2D trigger center pos2D
     * @param time action runtime in seconds
     * @param range trigger radius in inches
     */
    public ExampleAction (BioBuzzRobot robot, Pos2D pos2D, double time, double range){
        super(robot, pos2D, time, range);
    }

    /**
     * Standard Action, Triggers without condition, used in Teleop
     * @param robot the robot that the action will be triggered on
     */
    public ExampleAction (BioBuzzRobot robot){
        super(robot);
    }

    @Override
    public void execute() {
        // This is where you would put the code that you want to run when this action is executed
    }
    @Override
    public void process(){
        // This is where you would put the code that you want to iterate while this action is being processed
    }
    @Override public boolean completion(){
        // This is where you would put the code that you want to run to check if this action is complete
        return true; // Action is complete
    }
    @Override
    public boolean interrupt(){
        // This is the code that will be run when another action needs to interrupt this one, and it is still running.
        // Put code here that will safely make this action ready for whatever comes next
        return true; 
        //once this returns true, it is safe to transition. If condition is unsafe for transition,
        // return false and this method will be called again until it is safe.
    }
    @Override public void end(){
        // This is where you would put the code that you want to run when this action is completed
    }
}

