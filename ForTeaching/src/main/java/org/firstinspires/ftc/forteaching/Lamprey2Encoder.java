package org.firstinspires.ftc.forteaching;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.util.ElapsedTime;

// Started out as an actual device, but that was problematic for robot configuration :(
public class Lamprey2Encoder /*extends AnalogInput */{
    private AnalogInput lamprey;
    /* Stuff for the encoder: I should encapsulate this stuff */
    private ElapsedTime lastRead;
    private double lastGoodAngle;
    // Values to detect the 0->360 swings
    private static final double badRange = 3;
    private static final double swingRange = 15;
    private static final double msRange = 50.0;
    // This was from my observations:
    private static final double maxVolts = 2.321;
    private boolean isBadRead(double angle) {
        // TODO: Make this check to see if the value is physically possible based on time
        // First: Are we in the 'bad range' where smoothing might screw things up?
        return (lastGoodAngle < badRange || lastGoodAngle > 360 - badRange)
                // Are we reporting a number that's outside of what we believe is good?
                && (angle > badRange + swingRange && angle < 360 - badRange - swingRange)
                // Have is been short enough that the range we read is not possible?
                && lastRead.milliseconds() < msRange;
    }

    public double getAngle() {
        double thing = lamprey.getVoltage() / maxVolts;
        double angle = (360 * thing) % 360; // % 360 "just in case"
        // The encoder tries to smooth across 0->360 changes, which results in
        // bad readings that are wildly spread across the range, but only for
        // values where the delta crosses from 0 to 360 degrees
        if (isBadRead(angle)) {
            // TODO: Should we just try to read the value again instead?
            angle = lastGoodAngle;
        } else {
            lastGoodAngle = angle;
        }
        lastRead.reset();
        return angle;
    }
    // Hopefully this will work from hwMap.get(...)
    public Lamprey2Encoder(HardwareMap hwMap, String name) {
        lamprey = hwMap.get(AnalogInput.class, name);
        lastRead = new ElapsedTime();
    }
}
