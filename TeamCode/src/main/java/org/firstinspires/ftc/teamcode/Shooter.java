package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }
}
