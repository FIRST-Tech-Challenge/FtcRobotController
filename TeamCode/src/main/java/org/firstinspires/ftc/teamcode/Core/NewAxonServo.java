package org.firstinspires.ftc.teamcode.Core;


import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class NewAxonServo {
    private CRServo axon;
    private AnalogInput encoder;
    private double lastReadPosition;
    private double sign = 1;
    private double encoderOffset;
    private double inverseEncoderOffset;
    private double ratio;

    public NewAxonServo(CRServo axon, AnalogInput encoder, boolean inversePower, boolean inverseEncoder, double encoderOffset, double ratio) {
        this.axon = axon;
        this.encoder = encoder;
        if (inversePower) {
            sign = -1;
        }
        if (inverseEncoder) {
            inverseEncoderOffset = 360;
        }

        this.encoderOffset = encoderOffset;
        this.ratio = ratio;
    }

    public void set(double power) {
        axon.set(power * sign);
    }

    public double get() {
        return axon.get() * sign;
    }

    public void readPos() {

        lastReadPosition = (Math.abs(inverseEncoderOffset - (((encoder.getVoltage() / 3.3 * 360) + encoderOffset))));
    }

    public double readAndGetVoltage() {
        return encoder.getVoltage();
    }

    public double readAndGetRawPos() {
        return (encoder.getVoltage() / 3.3);
    }

    public double getServoAngle() {
        return lastReadPosition;
    }

    // equal to mechanism angle
    public double getScaledPos() {
        return (lastReadPosition*ratio);
    }

    public void setEncoderOffset(double offset) {
        this.encoderOffset = offset;
    }
}