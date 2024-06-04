package org.rustlib.hardware;

public interface Encoder {
    int getPosition();

    void setPosition(int position);

    double getVelocity();

    void reset();
}
