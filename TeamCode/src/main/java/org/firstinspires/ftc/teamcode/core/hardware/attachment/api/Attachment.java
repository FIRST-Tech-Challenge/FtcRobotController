package org.firstinspires.ftc.teamcode.core.hardware.attachment.api;

public interface Attachment {
    void moveOnThisThread(double position, double speed);

    void move(double position, double speed);

    void move(double position);
}
