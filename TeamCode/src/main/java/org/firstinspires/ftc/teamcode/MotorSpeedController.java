package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorSpeedController {
    
    private DcMotorEx motor;
    double minRPM = -100;
    double maxRPM = 100;
    // data below for GolBilda 5200 series motors
    double TICKS_PER_REV = 28.0;
    double tpsToRPM = 60 / TICKS_PER_REV;  // conversion factor - defaults to gear ratio = 1

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
        // allow overspeed by default
        setLimits(-motorRPM * 1.5, motorRPM * 1.5);
        // look up ratio from data arrays and adjust tpsToRPM
        double ratio = findRatio(motorRPM);
        tpsToRPM *= ratio;
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
     * Sets the minimum and maximum limits for motor speed. 
     * Inputs beyond these limits will be capped at the limits.
     * If not set, defaults to min=1 and max=1
     * @param min has a lowest possible value of -6000 (fastest available motor)
     * @param max has a highest possible value of 6000 (fastest available motor)
     */
    public void setLimits(double min, double max){
        minRPM = Math.max(min, -6000);
        maxRPM = Math.min(max, 6000);
    }

    /**
     * Sets the power level of the motor - remains set until changed
     * Set a power level of zero to stop
     * @param power a double from -1.0 to 1.0 unless further limited by setLimits()
     */
    public void setRPM(double rpm){
        rpm = Math.min(rpm, maxRPM);
        rpm = Math.max(rpm, minRPM);
        double ticksPerSec = (rpm / tpsToRPM);
        motor.setVelocity(ticksPerSec);
    }

    public double getRPM(){
        double ticksPerSecond = motor.getVelocity();
        double rpm = ticksPerSecond * tpsToRPM;
        return rpm;
    }
}
