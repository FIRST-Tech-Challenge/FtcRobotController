package org.firstinspires.ftc.teamcode.hardware;

import java.util.HashSet;

import com.qualcomm.robotcore.hardware.*;

public abstract class Arm {
    protected HashSet<DcMotor> motors;
    protected HashSet<Servo> servo;

    public Arm(HashSet<DcMotor> motors, HashSet<Servo> servos) {
        this.motors = motors;

        this.servo = servos;
    }

    /**
     * Get all the DcMotors that are used by this arm system.
     * @return A set that contains every DcMotor included by this arm system.
     */
    public abstract HashSet<DcMotor> getAllMotors();
}