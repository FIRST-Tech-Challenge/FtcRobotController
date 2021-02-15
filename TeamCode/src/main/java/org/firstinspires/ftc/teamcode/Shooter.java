package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Shooter {
    //This class is for the shooter and will include two motors
    //This class will include methods for power adjustment and turning the power on and off

    /*Public OpMode Members.*/
    public DcMotor leftShooter  = null;
    public DcMotor rightShooter = null;

    HardwareMap hwMap = null;

    /* Constructor */
    public Shooter() {

    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftShooter = hwMap.get(DcMotor.class, "left_shooter");
        rightShooter = hwMap.get(DcMotor.class, "right_shooter");
        //define motor direction
        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotor.Direction.REVERSE);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftShooter.setPower(0);
        rightShooter.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * This method passes a power value to both shooter motors.
     * NOTE: At the time of writing this, a negative value will cause the motors to rotate outwards.
     * @param power
     */
    public void shooterPower(double power) {
        leftShooter.setPower(-power);
        rightShooter.setPower(power);
    }

    /**
     * Will return infinity if no sensor is installed
     * @return Current battery voltage
     */
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for(VoltageSensor sensor : hwMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    /**
     * Scales shooter power for power shots in autonomous depending on current battery power
     * @return
     */
    public double scalePowerShot() {
        if (getBatteryVoltage() > 13) {
            return -.825;
        } else if (getBatteryVoltage() <= 13 && getBatteryVoltage() > 12.45) {
            return -.875;
        } else if (getBatteryVoltage() <= 12.45) {
            return -.9;
        }
        return -.875;
    }
    /**
     * Scales shooter power for high goal in autonomous depending on current battery power
     * @return
     */
    public double scaleHighGoal() {
        if (getBatteryVoltage() > 13) {
            return -.875;
        } else if (getBatteryVoltage() <= 13 && getBatteryVoltage() > 12.45) {
            return -.93;
        } else if (getBatteryVoltage() <= 12.45) {
            return -.95;
        }
        return -.93;
    }
}
