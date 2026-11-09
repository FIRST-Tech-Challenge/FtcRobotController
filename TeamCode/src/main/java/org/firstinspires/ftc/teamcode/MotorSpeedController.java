package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorSpeedController {
    
    private DcMotorEx motor;
    // data below for GolBilda 5200 series motors
    double TICKS_PER_MOTOR_REV = 28.0;            // GoBilda 5202 series has 28 encoder ticks per motor rev
    double ticksPerShaftRev = TICKS_PER_MOTOR_REV;   

    /**
     * Creates an object that controls the power of a single DC motor
     * @param hm the hardware map inheirited from OpMode (should be passed as simply 'hardwareMap')
     * @param motorHardwareName the name of the motor as configured in the Driver Station
     * @param motorRPM the nameplate RPM of the motor
     */
    public MotorSpeedController(HardwareMap hm, String motorHardwareName, int motorRPM){
        motor = hm.get(DcMotorEx.class, motorHardwareName);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // look up ratio from data arrays and adjust tpsToRPM
        double ratio = findRatio(motorRPM);
        ticksPerShaftRev = TICKS_PER_MOTOR_REV * ratio;
    }

    private double findRatio(int rpm){
        int[] MOTOR_SPEEDS = {30, 117, 223, 312, 435, 1150, 1620, 6000};
        double[] GEAR_RATIOS = {200, 50.9, 26.9, 19.2, 13.7, 5.2, 3.7, 1};
        for (int i = 0; i < MOTOR_SPEEDS.length; i++) {
            if (MOTOR_SPEEDS[i] == rpm) return GEAR_RATIOS[i];
        }
        throw new RuntimeException(String.format("Invalid motor type: %d rpm",rpm));
    }

    public void setDirection(int vector){
        if (vector < 0){
            motor.setDirection(DcMotorSimple.Direction.REVERSE);       
        } else {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);       
        }
    }

    /**
     * Uses motor reverse power to force the motor to 0 rpm
     */
    public void brake(){
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setVelocity(0);
    }

    /**
     * Cuts power to the motor and allows it to freely coast
     */
    public void coast(){
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setVelocity(0);
    }

    /**
     * Sets the power level of the motor - remains set until changed
     * Set a power level of zero to allow motor to coast
     * @param rpm the targer rpm, with negative numbers giving reverse rotation
     * @return the current motor rpm
     */
    public double setRPM(double rpm){
        double ticksPerSec = ticksPerShaftRev * rpm / 60;
        motor.setVelocity(ticksPerSec);
        return getRPM();
    }

    public double getRPM(){
        double ticksPerSecond = motor.getVelocity();
        double rpm = ticksPerSecond * 60 / ticksPerShaftRev;
        return rpm;
    }
}
