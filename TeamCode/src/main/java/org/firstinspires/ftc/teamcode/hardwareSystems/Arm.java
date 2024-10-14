package org.firstinspires.ftc.teamcode.hardware;

import java.util.HashSet;

import com.qualcomm.robotcore.hardware.*;

/** @noinspection ALL */
public abstract class Arm {
    protected final HashSet<DcMotor> MOTORS;
    protected final HashSet<Servo> SERVOS;

    public Arm() {
        this.MOTORS = new HashSet<>();
        this.SERVOS = new HashSet<>();
    }

    public Arm(HashSet<DcMotor> motors, HashSet<Servo> servos) {
        this.MOTORS = motors;
        this.SERVOS = servos;
    }

    /**
     * Get all the DcMotors that are used by this arm system.
     * 
     * @return A set that contains every DcMotor included by this arm system.
     */
    public HashSet<DcMotor> getMotors() {
        return MOTORS;
    }
}