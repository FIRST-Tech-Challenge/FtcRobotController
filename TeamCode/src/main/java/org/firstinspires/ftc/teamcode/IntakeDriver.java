package org.firstinspires.ftc.teamcode;


public class IntakeDriver {

    private static final double MAX_MOTOR_SPEED = 312;
    private static final double MOTOR_SPEED = 120;

    private MotorSpeedController intakeMotor;

    public IntakeDriver(HardwareMap hm, String intakeMotorName) {
    
        intakeMotor = new MotorSpeedController(hm, intakeMotorName, MAX_MOTOR_SPEED);
        //TODO check rotation direction
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
