package org.firstinspires.ftc.teamcode.subsystems;

public interface Subsystem {
    void init();
    /** Called every loop. Must not block (no sleep or waiting loops). */
    void update();
    void stop();
}
