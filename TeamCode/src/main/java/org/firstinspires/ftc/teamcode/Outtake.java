package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    DcMotor liftMotor;
    Servo trapdoorServo;
    TouchSensor bottomLimit;

    int encoderTopLimit = 9700;

    boolean mailboxOpen = false;

    // Initiates the motors and servos we need for this subsystem.
    public Outtake(HardwareMap hwMap) {
        liftMotor = hwMap.get(DcMotor.class, "outtakeMotor");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        bottomLimit = hwMap.get(TouchSensor.class, "bottomLimit");
        
        trapdoorServo = hwMap.get(Servo.class, "trapdoorServo");
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // This functions uses one double input to drive the lift.
    public void driveLift(double power) {

        if (bottomLimit.isPressed() && power < 0.0) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(0.0);
        }
        else if (power > 0.0 && getLiftMotorPos() > encoderTopLimit) {
            liftMotor.setPower(0.0);
        } else if (mailboxOpen && power != 0.0) {
            closeMailbox();
        } else {
            liftMotor.setPower(power);
        }

    }

    public int getLiftMotorPos() {
       return liftMotor.getCurrentPosition();
    }

    public void resetLiftEncoders () {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void driveLiftUp(double power) {
        liftMotor.setPower(Math.abs(power));
    }

    public void driveLiftDown(double power) {
        liftMotor.setPower(-Math.abs(power));
    }

    public void openMailbox() {
        trapdoorServo.setPosition( 1.0);
        mailboxOpen = true;
    }

    public void closeMailbox() {
        trapdoorServo.setPosition(0.0);
        mailboxOpen = false;
    }

    // This function controls the trapdoor.
    // The first input is the button used to control the trap door.
    // The second input is the time the function uses to space out inputs.
    public void trapdoor(boolean button, ElapsedTime time) {
        if (button && time.time() > .50 && !mailboxOpen) {
            mailboxOpen = true;
            time.reset();
            trapdoorServo.setPosition(0.4);

        }
        else if (button && time.time() > .50 && mailboxOpen) {
            mailboxOpen = false;
            time.reset();
            trapdoorServo.setPosition(0.0);

        }
    }
}
