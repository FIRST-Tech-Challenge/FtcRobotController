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
        //Old linear version (flawed at extreme powers)
//        double scalar = 120;
//        return .108*(getBatteryVoltage()) - 1.265 - scalar/124.5;
//        double scalar = 142.606;
//        return -Math.cbrt((121050*getBatteryVoltage()/607117) + (Math.sqrt(Math.pow((242100*getBatteryVoltage()/607117 - 52.1392052),2) - 650.717989))/2 - 26.0696026)/3 - 0.843916411 + (1.81965764/Math.cbrt((121050*getBatteryVoltage()/607117) + (Math.sqrt(Math.pow((242100*getBatteryVoltage()/607117 - 52.1392052),2) - 650.717989))/2 - 26.0696026)) + .7;
        if (-1*.00554298*Math.pow(getBatteryVoltage(), 3) + .15788*Math.pow(getBatteryVoltage(), 2) - 1.17953*getBatteryVoltage() < -1)
            return -1;
        else
            return -1*.00554298*Math.pow(getBatteryVoltage(), 3) + .15788*Math.pow(getBatteryVoltage(), 2) - 1.17953*getBatteryVoltage();
    }

    public double scaleHighGoalDynamic() {
        //Old linear version (flawed at extreme powers)
        double scalar = 128;
        return .108*(getBatteryVoltage()) - 1.265 - scalar/124.5;
//        double scalar = 149.799;
//        return (7.315*Math.pow(getBatteryVoltage(),3)) + (18.519*Math.pow(getBatteryVoltage(),2)) + (14.469*getBatteryVoltage()) + 3.28 - (scalar/124.5);
    }

    public double getShooterPower() {
        return rightShooter.getPower();
    }
}
