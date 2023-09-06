package org.firstinspires.ftc.teamcode.subsystems.hardware;

public enum WheelPositions{
    FrontRight(0),
    RearRight(1),
    RearLeft(2),
    FrontLeft(3);
    public final int position;
    WheelPositions(int position) {
        this.position = position;
    }
}
