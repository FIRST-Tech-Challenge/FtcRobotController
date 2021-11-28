package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.core.ROBOT_DATA.*;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.MathFunctions;

// Checked for EE
public class Potentiometer {

    private AnalogInput potentiometer;
    private boolean breakable;
    private double currentValue;

    public Potentiometer(HardwareMap hwMap, String id) {
        potentiometer = hwMap.get(AnalogInput.class, id);
        this.breakable = false;
    }

    public Potentiometer(HardwareMap hwMap, String id, boolean breakable) {
        potentiometer = hwMap.get(AnalogInput.class, id);
        this.breakable = breakable;
    }

    /** This function MUST return 3.3 for this class to work*/
    public double getMaxVoltage() {
        return potentiometer.getMaxVoltage();
    }

    public void update() {
        double voltage = potentiometer.getVoltage();
        if (MathFunctions.epsEquals(voltage, 0)) {currentValue = 270d;}
        else {
            double output = (-234*( Math.sqrt(Math.pow(voltage, 2)-(1.1*voltage)+0.908) - (0.577*(voltage+1.65)))) / voltage;
            if (this.breakable) {
                if (output < 0) { throw new RuntimeException("Potentiometer voltage was greater than 3.3");}
                else if (output > 270) { throw new RuntimeException("Potentiometer voltage was less than 0");}
            }
            currentValue = MathFunctions.clamp(output, 270, 0);
        }
    }

    public double getAngleDegrees() {
        return this.currentValue;
    }

    public double getAngleRadians() {
        return Math.toRadians(getAngleDegrees());
    }
}
