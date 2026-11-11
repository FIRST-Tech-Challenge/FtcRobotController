/*
package org.firstinspires.ftc.teamcode.robot.actions;

import com.broombots.sweep.Classes.Action;
import com.broombots.sweep.Classes.Pos2D;

import org.firstinspires.ftc.teamcode.robot.BioBuzzRobot;

public class SetFlywheelSpeedAction extends Action {
    BioBuzzRobot robot;
    double flywheeelSpeed;
    double time;
    Pos2D position;
    public SetFlywheelSpeedAction (BioBuzzRobot robot, Pos2D position,double time, double speed){
        super(robot);
        this.robot=robot;
        this.flywheeelSpeed=speed;
        this.time=time;
        this.position = position;

    }


    @Override
    public void execute() {
        robot.getFlywheelPollen().setTargetSpeed(flywheeelSpeed);
    }
    @Override
    public void process(){
        // This is where you would put the code that you want to iterate while this action is being processed
    }
    @Override public boolean completion(){
        // This is where you would put the code that you want to run to check if this action is complete
        if(time >=2){
            return true;
        }
        return false; // Action is complete
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
 */
