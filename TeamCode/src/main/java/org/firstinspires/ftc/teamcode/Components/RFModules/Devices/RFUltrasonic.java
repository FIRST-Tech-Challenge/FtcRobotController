package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;

/**
 * William
 */
public class RFUltrasonic {
    private AnalogInput ultrasonicAnalog;
    private LED ultrasonicLED;
    private Line targetLine;
    double MAX_RANGE = 254;
    double lastEnabled = 0;

    double ULTRA_FACTOR = 90.48337;
    double ULTRA_ADJUSTMENT = 12.62465;

    double difference = 0;
    boolean detected = false;

    /**
     * Constructor
     * @param p_ultraAnalogName name of the ultrasonic analog input in the hardware map
     * @param p_ultraLEDName name of the ultrasonic LED in the hardware map
     */
    public RFUltrasonic(String p_ultraAnalogName, String p_ultraLEDName){
        ultrasonicAnalog = op.hardwareMap.get(AnalogInput.class, p_ultraAnalogName);
        ultrasonicLED = op.hardwareMap.get(LED.class, p_ultraLEDName);
        ultrasonicLED.enable(ultrasonicLED.isLightOn());
        targetLine = new Line(1,0,0, new Vector2d(0,-10), new Vector2d(0,10));
    }

    /**
     * Checks if there is an object in range of the ultrasonic sensor.
     * Logs whether it found an object or not.
     * Logs to RFUltrasonic & general logs.
     * Logs to least fine level.
     * Does not update a state machine.
     */
    public void check() {
        double robotDist = targetLine.distToLine();
        double ultraDist;
        if (op.getRuntime() - lastEnabled >= 0.05) {
            ultrasonicLED.enable(ultrasonicLED.isLightOn());
        }

        if (op.getRuntime() - lastEnabled >= 0.1) {

            if (ultrasonicLED.isLightOn()) {
                ultraDist = getDist();

                difference = robotDist - ultraDist;
                detected = ultraDist < robotDist;
            }

            ultrasonicLED.enable(ultrasonicLED.isLightOn());
            lastEnabled = op.getRuntime();
        }
    }

    public void setLine(Line p_targetLine) {
        targetLine = p_targetLine;
    }

    public double getDist() {
        return getVoltage()*ULTRA_FACTOR - ULTRA_ADJUSTMENT;
    }

    public double getVoltage() {
        return ultrasonicAnalog.getVoltage();
    }
    public boolean isDetected() {
        return detected;
    }
    public double getDifference() {
        return difference;
    }
}
