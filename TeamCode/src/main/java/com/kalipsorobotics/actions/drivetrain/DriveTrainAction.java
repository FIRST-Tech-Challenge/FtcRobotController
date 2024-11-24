package com.kalipsorobotics.actions.drivetrain;

import com.kalipsorobotics.PID.PIDController;
import com.kalipsorobotics.actions.Action;

public abstract class DriveTrainAction extends Action {

    public abstract PIDController getController();
    public abstract double getError();


}
