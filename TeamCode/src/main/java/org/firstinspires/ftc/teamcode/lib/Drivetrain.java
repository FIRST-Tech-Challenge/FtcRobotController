package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private DcMotorEx left_drive;
    private DcMotorEx right_drive;
    public Drivetrain(HardwareMap map) {
        this.left_drive = map.get(DcMotorEx.class, "left_drive");
        this.right_drive = map.get(DcMotorEx.class, "right_drive");

        this.right_drive.setPositionPIDFCoefficients(5);
        this.left_drive.setPositionPIDFCoefficients(5);
    }

    public void set_power(double left, double right) {
        this.left_drive.setPower(left);
        this.right_drive.setPower(right);
    }

    public void set_right_power(double power) {
        this.right_drive.setPower(power);
    }

    public void set_left_power(double power) {
        this.left_drive.setPower(power);
    }

    public void set_left_target_position(int pos) {
        this.left_drive.setTargetPosition(pos);
    }

    public void set_right_target_position(int pos) {
        this.right_drive.setTargetPosition(pos);
    }

    public void set_left_mode(DcMotor.RunMode mode) {
        this.left_drive.setMode(mode);
    }

    public void set_right_mode(DcMotor.RunMode mode) {
        this.right_drive.setMode(mode);
    }


    public double get_left_position() {
        return left_drive.getCurrentPosition();
    }

    public double get_right_position() {
        return right_drive.getCurrentPosition();
    }

    public boolean left_is_busy() {
        return left_drive.isBusy();
    }

    public boolean right_is_busy() {
        return right_drive.isBusy();
    }
}
