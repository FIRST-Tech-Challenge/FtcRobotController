package org.firstinspires.ftc.teamcode.hardware;

import java.util.HashSet;

public abstract class WheelsSystem {
    // A modifier for much power the wheels run with (0.0 - 1.0)
    protected double wheelPower = 1.0;
    
    public WheelsSystem() {

    }

    /**
     * Get all the DcMotors that are used by this wheels system.
     * @return A set that contains every DcMotor included by this wheels system.
     */
    public abstract HashSet<DcMotor> getAllMotors();
}
