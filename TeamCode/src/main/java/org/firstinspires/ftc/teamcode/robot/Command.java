package org.firstinspires.ftc.teamcode.robot;

public interface Command {
    void start();

    void update();

    void stop();

    boolean isCompleted();
}
