package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * William
 */
public class RFUltrasonic {
    private AnalogInput ultrasonic;

    double MAX_RANGE = 254;

    /**
     * Constructor
     * @param p_ultraName name of the ultrasonic in the hardware map
     */
    public RFUltrasonic(String p_ultraName){
        ultrasonic = op.hardwareMap.get(AnalogInput.class, p_ultraName);
    }

    /**
     * Checks if there is an object in range of the ultrasonic sensor.
     * Logs whether it found an object or not.
     * Logs to RFUltrasonic & general logs.
     * Logs to least fine level.
     * Does not update a state machine.
     */
    public boolean check(Line p_targetLine) {
        return ultrasonic.getVoltage() * MAX_RANGE < p_targetLine.distToLine();
    }

    public double getDist() {
        return ultrasonic.getVoltage() * MAX_RANGE;
    }

    public double getVoltage() {
        return ultrasonic.getVoltage();
    }
}
