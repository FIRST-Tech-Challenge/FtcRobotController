package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.core.ROBOT_DATA.*;

public class DriveMotor {

    private DcMotorEx motor;

    // Stored as encoder ticks
    private int lastPos;

    public DriveMotor(HardwareMap hwMap, String id, DcMotor.Direction direction) {
        this.motor = hwMap.get(DcMotorEx.class, id);
        this.motor.setDirection(direction);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastPos = 0;
    }

    public DcMotorEx getMotor() {
        return motor;
    }
    public void setPower(double power) {
        this.motor.setPower(power);
    }

    /** Returns the change in position since last call
     * @return  the change in ROTATIONS (not ticks)
     */
    public double getDPos() {
        int newPos = this.motor.getCurrentPosition();
        int ret = newPos - lastPos;
        lastPos = newPos;
        return ((double) ret) / TICKS_PER_REV;
    }

    /** Sets the velocity of the motor
     * @param velo  the velocity in ROTATIONS/SECOND
     */
    public void setVelocity(double velo) {
        this.motor.setVelocity(velo * TICKS_PER_REV);
    }
}
