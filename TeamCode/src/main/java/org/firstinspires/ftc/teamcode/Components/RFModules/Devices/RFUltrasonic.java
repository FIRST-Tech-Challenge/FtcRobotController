package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * William
 */
public class RFUltrasonic {
    private UltrasonicSensor ultrasonic;

    /**
     * Constructor
     * @param p_ultraName name of the ultrasonic in the hardware map
     */
    public RFUltrasonic(String p_ultraName){
        ultrasonic = op.hardwareMap.get(UltrasonicSensor.class, p_ultraName);
    }

    /**
     * Checks if there is an object in range of the ultrasonic sensor.
     * Logs whether it found an object or not.
     * Logs to RFUltrasonic & general logs.
     * Logs to least fine level.
     * Does not update a state machine.
     */
    public boolean check() {
        return true;
        //placeholder
    }

    //distance data, flip pin
}
