package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.List;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Pose;

/** Common lifecycle for normal routines and isolated simulator experiments. */
public interface RobotController {
    void start(int alliance,double power);
    void tick();
    void stop();
    boolean isRunning();
    Enum<?> getState();
    String status();
    Pose destination();
    List<Pose> route();
}
