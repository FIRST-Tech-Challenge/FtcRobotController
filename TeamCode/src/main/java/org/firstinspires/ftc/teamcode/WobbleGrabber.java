package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGrabber {
    //This class will control the wobble grabber which has 1 motor and 1 servo
    //This class will include methods for opening and closing gripper & moving the arm up and down

    /*Public OpMode Members.*/
    public DcMotor gripWrist  = null;
    public Servo gripServo = null;

    HardwareMap hwMap = null;

    DigitalChannel REVTouchBottom;

    /* Constructor */
    public WobbleGrabber() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        gripWrist = hwMap.get(DcMotor.class, "grip_wrist");
        gripServo = hwMap.get(Servo.class, "grip_servo");
        //define motor direction
        gripWrist.setDirection(DcMotor.Direction.REVERSE);

        gripWrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        gripWrist.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        gripWrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed sensors
        REVTouchBottom = hwMap.get(DigitalChannel.class, "Bottom_Touch");

        // set the digital channel to input.
        REVTouchBottom.setMode(DigitalChannel.Mode.INPUT);
    }

    public void lowerGripper(double power) {
        // if the digital channel returns true it's HIGH and the button is unpressed.
        if (REVTouchBottom.getState()) {
            gripWrist.setPower(power);//May need to be fixed for direction
            while (REVTouchBottom.getState());
            gripWrist.setPower(0);
        }
    }
    public void gripperPosition(double position) {
        gripServo.setPosition(position);
    }
}
