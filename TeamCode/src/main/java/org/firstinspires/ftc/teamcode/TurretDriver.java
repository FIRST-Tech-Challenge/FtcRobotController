package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class TurretDriver {
    
    private MotorSpeedController flywheelMotor;
    private ServoMotorController servoMotor;
    private double currentRPM;
    final double FLYWHEEL_RPM = 1240; // TODO figure out correct RPM value
    final double FLYWHEEL_TOLERENCE = 0.05;

    public TurretDriver(HardwareMap hm, String flywheelMotorName, String servoName) {
        this.flywheelMotor = new MotorSpeedController(hm, flywheelMotorName, 6000);
        this.servoMotor = new ServoMotorController(hm, servoName);
        //TODO check rotation direction
        this.flywheelMotor.setDirection(1);
        this.servoMotor.runServoTurret(true); // takes a double from 0.0 to 1.0, our servos max turn is 1800 degrees,
                                   // so every 0.2 is 360 degrees

                                   // 205 - 70 gear ratio (2.9286)
}


    public void turnTo(double degrees){
        //TODO use servo position driver to clamp the turret to stay within 360 degrees
        if (degrees > 360){
            degrees = 360;
        }
        if (degrees < 0){
            degrees = 0;
        }

    }

    public void turn(boolean dir) {
        servoMotor.runServoTurret(dir);
    }

    public boolean startFlywheel(){
        currentRPM = flywheelMotor.getRPM();
        flywheelMotor.setRPM(FLYWHEEL_RPM);
        
        return (currentRPM / FLYWHEEL_RPM <= FLYWHEEL_TOLERENCE);
    }

    public boolean stopFlywheel(){
        flywheelMotor.setRPM(0);
        return (flywheelMotor.getRPM() == 0);
    }

    public double setFlywheelRPM(double rpm){
        flywheelMotor.setRPM(rpm);
        return flywheelMotor.getRPM();
    }
}