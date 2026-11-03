package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorPowerController {
    
    private DcMotor motor;
    private double minPower = -1;
    private double maxPower = 1;

    /**
     * Creates an object that controls the power of a single DC motor
     * @param hm the hardware map inheirited from OpMode (should be passed as simply 'hardwareMap')
     * @param motorHardwareName the name of the motor as configured in the Driver Station
     */
    public MotorPowerController(HardwareMap hm, String motorHardwareName){
         motor = hm.get(DcMotor.class, motorHardwareName);
         motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setDirection(int vector){
        if (vector < 0){
            motor.setDirection(DcMotorSimple.Direction.REVERSE);       
        } else {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);       
        }
    }

    /**
     * Sets the minimum and maximum limits for motor power. 
     * Inputs beyond these limits will be capped at the limits.
     * If not set, defaults to min=-1 and max=1
     * @param min has a lowest possible value of -1 (100% reverse power)
     * @param max has a highest possible value of 1 (100% forward pwoer)
     */
    public void setLimits(double min, double max){
        minPower = Math.max(min, -1);
        maxPower = Math.min(max, 1);
    }

    /**
     * Sets the power level of the motor - remains set until changed
     * Set a power level of zero to stop
     * @param power a double from -1.0 to 1.0 unless further limited by setLimits()
     */
    public void setPower(double power){
        power = Math.min(power, maxPower);
        power = Math.max(power, minPower);
        motor.setPower(power);
    }
}
