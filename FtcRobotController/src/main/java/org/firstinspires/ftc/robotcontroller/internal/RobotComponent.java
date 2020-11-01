package org.firstinspires.ftc.robotcontroller.internal;

import org.firstinspires.ftc.robotcontroller.internal.robotBase;
public abstract class RobotComponent {
   public robotBase base;

    public RobotComponent (robotBase base){
        this.base = base;
    }

    public robotBase getRobotbase() {
        return base;
    }

    public abstract void stop();
}
