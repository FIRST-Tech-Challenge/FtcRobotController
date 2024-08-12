package org.rustlib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.rustlib.commandsystem.Subsystem;

public class PairedEncoder extends Subsystem implements Encoder {
    private final int polarity;
    DcMotor pairedMotor;
    ElapsedTime timer = new ElapsedTime();
    private int ticks = 0;
    private int offset = 0;
    private double lastPosition = 0;
    private double lastTimestamp = 0;
    private double ticksPerSecond = 0;
    private final double ticksToPosition;

    public PairedEncoder(DcMotor pairedMotor, boolean reversed, double ticksToPosition) {
        this.pairedMotor = pairedMotor;
        pairedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pairedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        polarity = reversed ? -1 : 1;
        this.ticksToPosition = ticksToPosition;
    }

    public PairedEncoder(DcMotor pairedMotor, double ticksToPosition) {
        this(pairedMotor, false, ticksToPosition);
    }

    public PairedEncoder(DcMotor pairedMotor, boolean reversed) {
        this(pairedMotor, reversed, 1.0);
    }

    public PairedEncoder(DcMotor pairedMotor) {
        this(pairedMotor, false, 1.0);
    }

    private int calculateTicks() {
        return (pairedMotor.getCurrentPosition() + offset) * polarity;
    }

    @Override
    public int getTicks() {
        return ticks;
    }

    public double getPosition() {
        return ticks * ticksToPosition;
    }

    public void setTicks(int ticks) {
        offset = -pairedMotor.getCurrentPosition() + ticks * polarity;
    }

    public void setPosition(double position) {
        setTicks((int) (position / ticksToPosition));
    }

    @Override
    public double ticksPerSecond() {
        return ticksPerSecond;
    }

    public double getVelocity() {
        return ticksPerSecond * ticksToPosition;
    }

    private double calculateTicksPerSecond() {
        double position = getTicks();
        double timestamp = timer.milliseconds();
        double ticksPerSecond = (position - lastPosition) / (timestamp - lastTimestamp);
        lastPosition = position;
        lastTimestamp = timestamp;
        return ticksPerSecond;
    }

    @Override
    public void reset() {
        offset = -pairedMotor.getCurrentPosition();
    }

    @Override
    public void periodic() {
        ticks = calculateTicks(); // This is done to ensure that the encoder value is being read in every loop, which allows the hubs to do bulk hardware reading
        ticksPerSecond = calculateTicksPerSecond();
    }
}
