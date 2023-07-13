package org.firstinspires.ftc.teamcode.Old.PowerPlay.Components;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

//CLAWEXTENSION STATE MACHINE:
//CLAW_EXTENDING, CLAW_EXTENDED_FULLY, CLAW_RETRACTING, CLAW_RETRACTED_FULLY, CLAW_EXTENDED_TO_CUSTOM_POSITION
//wrote out basically what state machines will look like with if statements & numbers to indicate state

//TODO: capitalize c in ClawExtension
public class ClawExtension {
    double clawMoveStart;
    final double CLAW_MOVE_TIME = 0.75;
    public enum ClawExtensionStates {
        CLAW_EXTENDED(false, "CLAW_EXTENDED"),
        CLAW_EXTENDING(false, "CLAW_EXTENDING"),
        CLAW_UNRETRACTED(false, "CLAW_EXTENDED_TO_POS"),
        CLAW_RETRACTING(false,"CLAW_RETRACTING"),
        CLAW_RETRACTED(true,"CLAW_RETRACTED");

        boolean status;
        String name;

        ClawExtensionStates(boolean value, String name) {
            this.status = value;
            this.name = name;
        }

        public void setStatus(boolean status) {
            this.status = status;
            if(status==true){
                for(int i=0;i<ClawExtensionStates.values().length;i++){
                    if(this.name != ClawExtensionStates.values()[i].name()){
                        ClawExtensionStates.values()[i].setStatus(false);
                    }
                }
            }
        }
    }
    RFServo clawExtendServo;
    final double INCHES_PER_POS = 10/1;
    public ClawExtension(){
        clawExtendServo = new RFServo("clawExtendServo", 1);
        logger.createFile("ClawExtensionLog", "Time Desc Value");
    }
    public void extendToPosition(double inches){
        //use rfservo setPosition for variable distance
        //input is inches that claw should extend in inches
        //set servo range & extend to it
        //log when & what range is set to

        clawExtendServo.setPosition(inches/INCHES_PER_POS);
        //only the first time you start moving set state
        if(!ClawExtensionStates.CLAW_EXTENDING.status) {
            ClawExtensionStates.CLAW_EXTENDING.setStatus(true);
            clawMoveStart=op.getRuntime();
            logger.log("ClawExtensionLog", " Claw started extending");
        }
        if(op.getRuntime()>clawMoveStart+CLAW_MOVE_TIME) {
            ClawExtensionStates.CLAW_UNRETRACTED.setStatus(true);
            logger.log("ClawExtensionLog", " Claw Extended to: " + inches + " in.");
        }
    }
    public void extendClaw(){
        //no input TODO: maybe a seperate function for setPosition so you have variable distance
        //state of extendClaw has to be false(retracted state)
        //set servo position
        //set state of claw extended to true
        //log when extension starts & when movement ends
        if(!ClawExtensionStates.CLAW_EXTENDED.status) {
            clawExtendServo.setPosition(1.0);
            if (!ClawExtensionStates.CLAW_EXTENDING.status) {
                ClawExtensionStates.CLAW_EXTENDING.setStatus(true);
                clawMoveStart = op.getRuntime();
                logger.log("ClawExtensionLog", " Claw started extending");
            }
            if (op.getRuntime() > clawMoveStart + CLAW_MOVE_TIME) {
                ClawExtensionStates.CLAW_EXTENDED.setStatus(true);
                logger.log("ClawExtensionLog", " Claw fully extended");
            }
        }
    }
    public void retractClaw(){
        //no input
        //state of extendClaw has to be true(extended state)
        //set servo position
        //set state of claw extended to false
        //log when extension starts & when movement ends
        if(ClawExtensionStates.CLAW_RETRACTED.status)
        clawExtendServo.setPosition(0.0);
        if(!ClawExtensionStates.CLAW_RETRACTING.status) {
            ClawExtensionStates.CLAW_RETRACTING.setStatus(true);
            clawMoveStart=op.getRuntime();
            logger.log("ClawExtensionLog", " Claw started retracting");
        }
        if(op.getRuntime()>clawMoveStart+CLAW_MOVE_TIME) {
            ClawExtensionStates.CLAW_RETRACTED.setStatus(true);
            logger.log("ClawExtensionLog", " Claw fully retracted");
        }
    }
}
