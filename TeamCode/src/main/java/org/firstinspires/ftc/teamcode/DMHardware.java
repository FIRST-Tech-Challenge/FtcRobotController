package org.firstinspires.ftc.teamcode;

// Importing all of the packages

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DMHardware {

    // Instantiating the hardware classes
    public DcMotor leftMotor, rightMotor, armMotor;
    public Servo claw;
    public TouchSensor armLimiter;

    public boolean runThisWithEncoders;

    HardwareMap hwMap;
    ElapsedTime timer = new ElapsedTime();


    // Methods...

    public DMHardware(boolean runThisWithEncoders) {
        this.runThisWithEncoders = runThisWithEncoders;
    }

    public void initTeleOpIMU(HardwareMap hwMap) {
        this.hwMap = hwMap;

        timer.reset();



        // Adding variable names to the hardware...

        claw = hwMap.servo.get("claw");
        claw.setPosition(0.2);

        armLimiter = hwMap.touchSensor.get("lift_limiter");

        leftMotor = hwMap.dcMotor.get("left_motor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        rightMotor = hwMap.dcMotor.get("right_motor");

        armMotor = hwMap.dcMotor.get("lift_motor");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // Method to set the power of all of the motors...

    public void setPowerOfAllMotorsTo(double speed) {
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
    }

    // Method to get current time...
    public double getTime() {

        return timer.time();
    }

}
