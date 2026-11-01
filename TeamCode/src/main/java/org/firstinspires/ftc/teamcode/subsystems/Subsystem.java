package org.firstinspires.ftc.teamcode.subsystems;

public interface Subsystem {
    boolean TipDetect(double startCluster, double endCluster);

    void init();
    /** Called every loop. Must not block (no sleep or waiting loops). */
    void update();
    void stop();
}
