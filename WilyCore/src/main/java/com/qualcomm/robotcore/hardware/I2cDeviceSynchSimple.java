package com.qualcomm.robotcore.hardware;

public interface I2cDeviceSynchSimple extends HardwareDevice, I2cAddrConfig
{
    byte read8();
    byte read8(int ireg);
    byte[] read(int creg);
    byte[] read(int ireg, int creg);
    TimestampedData readTimeStamped(int creg);
    TimestampedData readTimeStamped(int ireg, int creg);
    void write8(int bVal);
    void write8(int ireg, int bVal);
    void write(byte[] data);
    void write(int ireg, byte[] data);
    void write8(int bVal, I2cWaitControl waitControl);
    void write8(int ireg, int bVal, I2cWaitControl waitControl);
    void write(byte[] data, I2cWaitControl waitControl);
    void write(int ireg, byte[] data, I2cWaitControl waitControl);
    void waitForWriteCompletions(I2cWaitControl waitControl);
    void enableWriteCoalescing(boolean enable);
    boolean isWriteCoalescingEnabled();
    boolean isArmed();
    void setLogging(boolean enabled);
    boolean getLogging();
    void setLoggingTag(String loggingTag);
    String getLoggingTag();
    void engage(); // ### Shouldn't be here but couldn't find where it goes
}
