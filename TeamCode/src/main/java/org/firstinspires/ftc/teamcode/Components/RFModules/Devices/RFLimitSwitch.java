package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;

/**
 * Warren
 * This class is made for limit switches, makes it easier to use with built in functions
 */
public class RFLimitSwitch {
    private DigitalChannel limitSwitch;
    private double lastSwitchTime = -100;
    private boolean mode = false, pressed = false;

    private String name;

    /**
     * initializes the hardware
     * Logs that function is called and that hardware is initialized to general surface level
     * @param p_name the config name of the limit switch
     */
    public RFLimitSwitch(String p_name){
        LOGGER.setLogLevel(RFLogger.Severity.ALL);
        name = p_name;
        LOGGER.log("RFLimitSwitch(): intializing limit switch: " +name);
        limitSwitch = op.hardwareMap.get(DigitalChannel.class, name);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        mode=false;
    }

    /**
     * Checks the state of the switch and logs if it has changed since the last check to finest general log
     * @return if switch is pressed
     */
    public boolean isSwitched() {
        LOGGER.setLogLevel(RFLogger.Severity.FINEST);
        LOGGER.log("RFLimitSwitch:" +name + "isSwitched()");
        var newState = limitSwitch.getState()||mode;
        if(newState!=pressed){
            LOGGER.setLogLevel(RFLogger.Severity.FINE);
            LOGGER.log("RFLimitSwitch:" +name + "Switch State Changed : switched" + newState);
        }
        pressed = newState;
        LOGGER.setLogLevel(RFLogger.Severity.FINEST);
        LOGGER.log("RFLimitSwitch:" +name + "Switch State: switched" + newState);
        return pressed;
    }

    /**
     * turn the sensor on or off, when mode is set to true the switch will always think it pressed, logs to surface general log
     * @param p_mode if you want the sensor to always return true or not
     */
    public void setMode(boolean p_mode){
        if(time-lastSwitchTime>0.4) {
            LOGGER.setLogLevel(RFLogger.Severity.ALL);
            LOGGER.log("RFLimitSwitch.setMode():" +name + " : switch mode: " + p_mode);
            lastSwitchTime=time;
            mode = p_mode;
        }
    }

    /**
     * read the sensor mode, logs to finest general log
     * @return the mode of the sensor
     */
    public boolean getMode(){
        LOGGER.setLogLevel(RFLogger.Severity.FINEST);
        LOGGER.log("RFLimitSwitch.getMode():" +name + " : switch mode: " + mode);
        return mode;
    }

}
