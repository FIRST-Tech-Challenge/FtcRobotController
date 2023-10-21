package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * William
 */
public class RFUltrasonic {
    private AnalogInput ultrasonic;

    double MAX_RANGE = 254;

    double ULTRA_FACTOR = 90.48337;
    double ULTRA_ADJUSTMENT = 12.62465;

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
        double dist = p_targetLine.distToLine();
        packet.put("distTOline", dist);
        return ultrasonic.getVoltage() * ULTRA_FACTOR - ULTRA_ADJUSTMENT < dist;
    }

    public double getDist() {
        return ultrasonic.getVoltage()*ULTRA_FACTOR - ULTRA_ADJUSTMENT;
    }

    public double getVoltage() {
        return ultrasonic.getVoltage();
    }
}
