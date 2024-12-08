package com.kalipsorobotics.actions.drivetrain;

import com.kalipsorobotics.PID.PIDController;
import com.kalipsorobotics.actions.Action;

public abstract class DriveTrainAction extends Action {

    public abstract PIDController getPidController();
    public abstract double getRemainingDistance();

    public abstract double getTarget();

    public abstract double getDuration();

    public abstract double getOvershoot();

    public abstract void setPidController(PIDController controller);
}
