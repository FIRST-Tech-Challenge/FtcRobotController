package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeDriver {

    private static final int MAX_MOTOR_SPEED = 312;
    private static final int MOTOR_SPEED = 120;

    private MotorSpeedController intakeMotor;

    public IntakeDriver(HardwareMap hm, String intakeMotorName) {
    
        intakeMotor = new MotorSpeedController(hm, intakeMotorName, MAX_MOTOR_SPEED);
        intakeMotor.setDirection(1);
    }

    public void run() {
        intakeMotor.setRPM(MOTOR_SPEED);
    }

    public void stop() {
        intakeMotor.setRPM(0);
    }

    public void reverse() {
        intakeMotor.setRPM(-MOTOR_SPEED);

    }
}
