package org.firstinspires.ftc.teamcode.hardwareSystems;

import java.util.HashSet;

import com.qualcomm.robotcore.hardware.*;

public abstract class Arm {
    protected final HashSet<DcMotor> MOTORS;
    protected final HashSet<Servo> SERVOS;
    protected final MotorType MOTOR_TYPE;

    public Arm(HashSet<DcMotor> motors, HashSet<Servo> servos) {
        this(motors, servos, MotorType.TETRIX_TORQUENADO);
    }

    public Arm(HashSet<DcMotor> motors, HashSet<Servo> servos, MotorType motorType) {
        this.MOTORS = motors;
        this.SERVOS = servos;

        this.MOTOR_TYPE = motorType;
    }

    /**
     * Get all the {@code DcMotor}s that are included in this arm system.
     *
     * @return A {@code HashSet} that contains every DcMotor included in this arm system.
     */
    public HashSet<DcMotor> getMotors() {
        return MOTORS;
    }

    /**
     * Get all the {@code Servo}s that are included in this arm system.
     *
     * @return A {@code HashSet} that contains every Servo included in this arm system.
     */
    public HashSet<Servo> getServos() {
        return SERVOS;
    }

    public MotorType getMotorType() {
        return MOTOR_TYPE;
    }
}