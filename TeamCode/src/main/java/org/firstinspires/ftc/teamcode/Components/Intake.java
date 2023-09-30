package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFBreakBeam;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLimitSwitch;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
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
    private double pixelCount =0;

    /**
     * initializes all the hardware, logs that hardware has been initialized
     */
    public Intake(){
        super("intakeMotor",true);
        LOGGER.setLogLevel(RFLogger.Severity.ALL);
        LOGGER.log("Intake() : Initializing Intake Motor and intake sensors!");
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
            LOGGER.setLogLevel(RFLogger.Severity.ALL);
            LOGGER.log("IntakeStates.setStateTrue() : Intake state changed to: "+this.name());
            this.state = true;
        }
        public boolean getState(){
            return this.state;
        }
    }

    /**
     * Sets intake power to INTAKE_POWER, logs that the robot is intaking to general and intake surface level
     */
    public void intake(){
        LOGGER.setLogLevel(RFLogger.Severity.ALL);
        LOGGER.log("Intake.intake() : starting intake, power : "+INTAKE_POWER);
        setPower(INTAKE_POWER);
    }
    /**
     * Sets intake power to REVERSE_POWER, logs that the robot is reversing to general and intake surface level
     */
    public void reverseIntake(){
        LOGGER.setLogLevel(RFLogger.Severity.ALL);
        LOGGER.log("Intake.reverseIntake() : reversing intake, power : "+REVERSE_POWER);
        setPower(REVERSE_POWER);
    }
    /**
     * Sets intake power 0, logs that intake is stopped to general and intake surface level
     */
    public void stopIntake(){
        LOGGER.setLogLevel(RFLogger.Severity.ALL);
        LOGGER.log("Intake.stopIntake() : stopping intake, power : "+0);
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
        LOGGER.setLogLevel(RFLogger.Severity.FINEST);
        LOGGER.log("Intake.countPixels() : count : "+pixelCount + " --> " + count);
        if(count!=pixelCount){
            LOGGER.setLogLevel(RFLogger.Severity.ALL);
            LOGGER.log("Intake.countPixels() : PIXEL COUNT CHANGED, count : "+pixelCount + " --> " + count);
            pixelCount=count;
        }
        return count;
    }


    /**
     * updates the state machine, log in general and intake surface
     * updates sensor information, triggers following action to reverse/stop intaking
     */
    public void update(){
        LOGGER.setLogLevel(RFLogger.Severity.FINEST);
        LOGGER.log("Intake.update()");
        double power = this.getPower();
        LOGGER.setLogLevel(RFLogger.Severity.FINEST);
        LOGGER.log("intake power:" + power);
        if(power>0){
            IntakeStates.INTAKING.setStateTrue();
        }
        else if(power<0){
            IntakeStates.REVERSING.setStateTrue();
        }else{
            IntakeStates.STOPPED.setStateTrue();
        }
        if(countPixels()==2 && IntakeStates.INTAKING.state){
            LOGGER.setLogLevel(RFLogger.Severity.ALL);
            LOGGER.log("2 PIXELS REACHED STOPPING INTAKE");
            stopIntake();
        }
    }
}
