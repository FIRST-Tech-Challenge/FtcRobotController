package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Warren
 * This class is made for limit switches, makes it easier to use with built in functions
 */
public class RFLimitSwitch {
    private DigitalChannel limitSwitch;
    private double lastSwitchTime = -100;
    private boolean mode = false, pressed = false;
    public RFLimitSwitch(String name){
        limitSwitch = op.hardwareMap.get(DigitalChannel.class, name);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        mode=false;
    }

    /**
     * Checks the state of the switch and logs if it has changed since the last check to finest general log
     * @return if switch is pressed
     */
    public boolean isSwitched() {
        if(limitSwitch.getState()||mode!=pressed){
            //log
        }
        pressed = limitSwitch.getState()||mode;
        return pressed;
    }

    /**
     * turn the sensor on or off, when mode is set to true the switch will always think it pressed, logs to surface general log
     * @param p_mode if you want the sensor to always return true or not
     */
    public void setMode(boolean p_mode){
        if(time-lastSwitchTime>0.4) {
            lastSwitchTime=time;
            mode = p_mode;
        }
    }

    /**
     * read the sensor mode, logs to finest general log
     * @return the mode of the sensor
     */
    public boolean getMode(){
        return mode;
    }

}
