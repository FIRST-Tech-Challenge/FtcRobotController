package org.rustlib.hardware;

public interface Encoder {
    int getTicks();

    void setTicks(int ticks);

    double ticksPerSecond();

    void reset();
}
