package org.firstinspires.ftc.teamcode.utils.motors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;

public class HDrive {

    private final TetrixMotor rightMotor;
    private final TetrixMotor leftMotor;
    private final TetrixMotor centerMotor;
    private final Telemetry TELEMETRY;

    public HDrive(Telemetry telemetry, HardwareMap hardware) {
        rightMotor = new TetrixMotor(telemetry, hardware, hardware.appContext.getString(R.string.RIGHT_DRIVE_1), hardware.appContext.getResources().getInteger(R.integer.rd1_offset));
        leftMotor = new TetrixMotor(telemetry, hardware, hardware.appContext.getString(R.string.LEFT_DRIVE_1), hardware.appContext.getResources().getInteger(R.integer.ld1_offset));
        centerMotor = new TetrixMotor(telemetry, hardware, hardware.appContext.getString(R.string.CNTR_DRIVE_1), hardware.appContext.getResources().getInteger(R.integer.cd1_offset));
        TELEMETRY = telemetry;
    }

    public void move(double distance, double speed, double timeout) {
        MotorGroup motorGroup = new MotorGroup(new TetrixMotor[]{rightMotor, leftMotor, centerMotor}, TELEMETRY);
        motorGroup.move(distance, speed, timeout);
    }


}
