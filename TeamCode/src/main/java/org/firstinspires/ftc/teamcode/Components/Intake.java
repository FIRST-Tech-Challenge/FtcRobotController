package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFBreakBeam;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLimitSwitch;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.openftc.easyopencv.LIFO_OpModeCallbackDelegate;

/**
 * Warren
 * Class to contain flipping intake and associated functions
 */
public class Intake extends RFMotor {
    RFBreakBeam breakBeam;
    RFLimitSwitch limitSwitch;

    private final double INTAKE_POWER = 0.6;
    private final double REVERSE_POWER=-0.3;

    private boolean full = false;

    /**
     * initializes all the hardware, logs that hardware has been initialized
     */
    public Intake(){
        super("intakeMotor",true);
        breakBeam = new RFBreakBeam();
        limitSwitch = new RFLimitSwitch("intakeSwitch");
    }

    /**
     * possible states of intake
     */
    public enum IntakeStates{
        STOPPED(true),
        INTAKING(false),
        REVERSING(false);


        boolean state;
        IntakeStates(boolean p_state){
            state = p_state;
        }

        /**
         * sets current state to true, logs that this state is true in general and intake surface
         */
        public void setStateTrue() {
            for(int i = 0; i< IntakeStates.values().length; i++){
                IntakeStates.values()[i].state=false;
            }
            this.state = true;
        }
    }

    /**
     * Sets intake power to INTAKE_POWER, logs that the robot is intaking to general and intake surface level
     */
    public void intake(){
        setPower(INTAKE_POWER);
    }
    /**
     * Sets intake power to REVERSE_POWER, logs that the robot is reversing to general and intake surface level
     */
    public void reverseIntake(){
        setPower(REVERSE_POWER);
    }
    /**
     * Sets intake power 0, logs that intake is stopped to general and intake surface level
     */
    public void stopIntake(){
        setPower(0);
    }

    /**
     * count number of pixels using sensor, log to intake and general surface level when change
     * @return number of pixels in intake
     */
    public int countPixels(){
        int count=0;
        if(limitSwitch.isSwitched()){
            count++;
//            break beam here
            if(true){
                count++;
            }
        }
        return count;
    }


    /**
     * updates the state machine, log in general and intake surface
     * updates sensor information, triggers following action to reverse/stop intaking
     */
    public void update(){
        double power = this.getPower();
        if(power>0){
            IntakeStates.INTAKING.setStateTrue();
        }
        else if(power<0){
            IntakeStates.REVERSING.setStateTrue();
        }else{
            IntakeStates.STOPPED.setStateTrue();
        }
        if(countPixels()==2&& IntakeStates.INTAKING.state){
            stopIntake();
        }
    }
}
