package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * William
 */
public class RFUltrasonic {
    private UltrasonicSensor ultrasonic;

    private double MAX_RANGE = 30;

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
        return checkDist() < MAX_RANGE;
    }

    public double checkDist() {
        return ultrasonic.getUltrasonicLevel();
    }

    public void flipPin() {
        ultrasonic.close();
    }
}
