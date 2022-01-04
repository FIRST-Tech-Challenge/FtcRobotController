package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Shooter {
    //This class is for the shooter and will include one motor and one servo
    //This class will include methods for power adjustment and turning the power on and off

    /*Public OpMode Members.*/
    public DcMotor shooter  = null;
    public Servo shooterSwitch = null;

    HardwareMap hwMap = null;

    /* Constructor */
    public Shooter() {

    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        shooter = hwMap.get(DcMotor.class, "shooter");
        shooterSwitch = hwMap.get(Servo.class,"shooter_switch");
        //define motor direction
        shooter.setDirection(DcMotor.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        shooter.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set initial positions of servos
        shooterSwitch.setPosition(1);

    }

    /**
     * This method passes a power value to the shooter motor.
     * NOTE: At the time of writing this, a negative value will cause the motor to rotate outwards.
     * @param power
     */
    public void shooterPower(double power) {
        shooter.setPower(-power);
    }

    /**
     * This method sets the position of the servo that sends rings to the shooter
     * @param position
     */
    public void switchPosition(double position) {
        shooterSwitch.setPosition(position);
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
            return -.83;
        } else if (getBatteryVoltage() <= 13 && getBatteryVoltage() > 12.45) {
            return -.885;
        } else if (getBatteryVoltage() <= 12.45) {
            return -.91;
        }
        return -.885;
    }
    /**
     * Scales shooter power for high goal in autonomous depending on current battery power
     * @return
     */
    public double scaleHighGoal() {
        if (getBatteryVoltage() > 13) {
            return -.88;
        } else if (getBatteryVoltage() <= 13 && getBatteryVoltage() > 12.45) {
            return -.94;
        } else if (getBatteryVoltage() <= 12.45) {
            return -.96;
        }
        return -.94;
    }

    public double scalePowerShotDynamic() {
        //return .0446064*getBatteryVoltage() - 1.40412;
        return .0607178*getBatteryVoltage() - 1.59558;
    }

    public double scaleHighGoalDynamic() {
        //return .0446064*getBatteryVoltage() - 1.46412;
        return .0607178*getBatteryVoltage() - 1.63988;
    }

    public double getShooterPower() {
        return shooter.getPower();
    }
}
