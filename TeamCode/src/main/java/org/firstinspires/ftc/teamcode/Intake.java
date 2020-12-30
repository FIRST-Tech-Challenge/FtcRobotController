package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    //This class is for the intake and will contain two motors
    //This class will include methods to turn the intake on and off and lower the intake system to hit a touch sensor

    /*Public OpMode Members.*/
    public DcMotor intake  = null;
    public DcMotor transition = null;
    public Servo   intakeLatch = null;

    HardwareMap hwMap = null;

    /* Constructor */
    public Intake() {

    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        intake = hwMap.get(DcMotor.class, "intake");
        transition = hwMap.get(DcMotor.class, "transition");
        intakeLatch = hwMap.get(Servo.class,"intake_latch");
        //define motor direction
        intake.setDirection(DcMotor.Direction.REVERSE);
        transition.setDirection(DcMotor.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transition.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        intake.setPower(0);
        transition.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transition.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set servo to the closed position
        intakeLatch.setPosition(1);
    }

    /**
     * This method passes a power to the intake and transition motors to turn them on and off.
     * @param power
     */
    public void intakePower(double power) {
        intake.setPower(power);
        transition.setPower(power);
    }

    /**
     * This method opens the latch holding the intake allowing it to fall
     */
    public void lowerIntake() {
        intakeLatch.setPosition(.5);
    }

}
