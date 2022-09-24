package org.firstinspires.ftc.teamcode.Components;
public class ClawExtension {
    public ClawExtension(){
        // hardware mapping
    }
    public void extendToPosition(double inches){
        //use rfservo setRange for variable distance
        //input is inches that claw should extend in inches
        //set servo range
        //log when & what range is set to
    }
    public void extendClaw(){
        //no input
        //state of extendClaw has to be false(retracted state)
        //set servo position
        //set state of claw extended to true
        //log when extension starts & when movement ends
    }
    public void retractClaw(){
        //no input
        //state of extendClaw has to be true(extended state)
        //set servo position
        //set state of claw extended to false
        //log when extension starts & when movement ends
    }
}
