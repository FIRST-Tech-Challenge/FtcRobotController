package org.firstinspires.ftc.robotcontroller.internal;

import org.firstinspires.ftc.robotcontroller.internal.robotBase;
public abstract class RobotComponent {
   protected robotBase base;

    public RobotComponent (robotBase base){
        this.base = base;
    }

    public final robotBase base()
    {

        return base;
    }

    public abstract void stop();
}
