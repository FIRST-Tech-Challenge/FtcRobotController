package org.firstinspires.ftc.robotcontroller.internal;

import org.firstinspires.ftc.robotcontroller.internal.robotBase;
public abstract class RobotComponent {
    robotBase robotbase;

    public RobotComponent (robotBase robotbase){
        this.robotbase = robotbase;
    }

    public robotBase getRobotbase() {
        return robotbase;
    }

    public abstract void stop();
}
