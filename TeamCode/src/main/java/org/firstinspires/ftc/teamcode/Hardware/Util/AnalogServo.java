package org.firstinspires.ftc.teamcode.Hardware.Util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

public class AnalogServo {
    private Servo servo;
    private AnalogInput encoder;

    public AnalogServo(Servo servo, AnalogInput encoder) {
        this.servo = servo;
        this.encoder = encoder;
    }

    public void setPos(double position) {
        servo.setPosition(position);
    }

    public double getPos() {
        return voltsToDegrees(encoder.getVoltage());
    }

    private double getCommandedPos() {
        return servo.getPosition();
    }

    private double voltsToDegrees(double volts) {
        return (volts/3.3) * 360;
    }


}
