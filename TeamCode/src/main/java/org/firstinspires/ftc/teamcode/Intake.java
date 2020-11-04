package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Intake {
    //This class is for the intake and will contain two motors
    //This class will include methods to turn the intake on and off and lower the intake system to hit a touch sensor

    /*Public OpMode Members.*/
    public DcMotor intake  = null;
    public DcMotor intakeWrist = null;

    HardwareMap hwMap = null;

    DigitalChannel REVTouchBottom;

    /* Constructor */
    public Intake() {

    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        intake = hwMap.get(DcMotor.class, "intake");
        intakeWrist = hwMap.get(DcMotor.class, "intake_wrist");
        //define motor direction
        intake.setDirection(DcMotor.Direction.REVERSE);
        intakeWrist.setDirection(DcMotor.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeWrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        intake.setPower(0);
        intakeWrist.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeWrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed sensors
        REVTouchBottom = hwMap.get(DigitalChannel.class, "Bottom_Touch");

        // set the digital channel to input.
        REVTouchBottom.setMode(DigitalChannel.Mode.INPUT);
    }

    public void intakePower(double power) {
        intake.setPower(power);
    }
    public void lowerIntake(double power) {
        // if the digital channel returns true it's HIGH and the button is unpressed.
        if (REVTouchBottom.getState()) {
            intakeWrist.setPower(power);//May need to be fixed for direction
            while (REVTouchBottom.getState());
            intakeWrist.setPower(0);
        }
    }

}
