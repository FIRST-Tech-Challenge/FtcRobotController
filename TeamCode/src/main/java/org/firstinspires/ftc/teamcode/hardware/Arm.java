package org.firstinspires.ftc.teamcode.hardware;

import java.util.HashSet;

import com.qualcomm.robotcore.hardware.*;

public abstract class Arm {
    protected DcMotor[] motors;

    public Arm(DcMotor ...motors) {
        this.motors = motors;
    }

    /**
     * Get all the DcMotors that are used by this arm system.
     * @return A set that contains every DcMotor included by this arm system.
     */
    public abstract HashSet<DcMotor> getAllMotors();
}