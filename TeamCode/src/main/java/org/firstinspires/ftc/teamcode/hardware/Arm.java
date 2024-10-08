package org.firstinspires.ftc.teamcode.hardware;

import java.util.HashSet;

import com.qualcomm.robotcore.hardware.*;

public abstract class Arm {
    protected HashSet<DcMotor> motors;
    protected HashSet<Servo> servos;

    public Arm() {
        this.motors = new HashSet<>();
        this.servos = new HashSet<>();
    }

    public Arm(HashSet<DcMotor> motors, HashSet<Servo> servos) {
        this.motors = motors;
        this.servos = servos;
    }

    /**
     * Get all the DcMotors that are used by this arm system.
     * 
     * @return A set that contains every DcMotor included by this arm system.
     */
    public HashSet<DcMotor> getMotors() {
        return motors;
    }
}