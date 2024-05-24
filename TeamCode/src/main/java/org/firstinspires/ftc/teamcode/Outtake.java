package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    DcMotor liftMotor;
    Servo trapdoorServo;
    TouchSensor bottomLimit;

    boolean trapToggle = true;
    int encoderTopLimit = 9700;

    boolean mailboxOpen = false;

    // Initiates the motors and servos we need for this subsystem.
    public Outtake(HardwareMap hwMap) {
        liftMotor = hwMap.get(DcMotor.class, "outtakeMotor");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        
        trapdoorServo = hwMap.get(Servo.class, "trapdoorServo");
        bottomLimit = hwMap.get(TouchSensor.class, "bottomLimit");

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // This functions uses one double input to drive the lift.
    public void driveLift(double power) {
        if (bottomLimit.isPressed() && power < 0.0) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(0.0);
            //closeMailbox();
        }
        else if (power > 0.0 && getLiftMotorPos() > encoderTopLimit) {
            liftMotor.setPower(0.0);
        } else if (mailboxOpen && power != 0.0) {
            closeMailbox();
        } else {
            liftMotor.setPower(power);
        }
    }

    // This function controls the trapdoor.
    // The first input is the button used to control the trap door.
    // The second input is the time the function uses to space out inputs.
    public void trapdoor(boolean button, ElapsedTime time) {
        if (button && time.time() > .25 && !trapToggle) {
            trapToggle = true;
            time.reset();
            trapdoorServo.setPosition(0.4);
            mailboxOpen = false;

        }
        else if (button && time.time() > .25 && trapToggle) {
            trapToggle = false;
            time.reset();
            trapdoorServo.setPosition(0.0);
            mailboxOpen = true;

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
        trapdoorServo.setPosition(0.4);
        mailboxOpen = false;
    }

    public String getMailboxStatus() {
        String status;
        if (mailboxOpen) {
            status = "Open";
        }
        else {
            status = "Closed";
        }

        return status;
    }

    public String getBottomLimitStatus() {
        String status;
        if (bottomLimit.isPressed()) {
            status = "Pressed";
        }
        else {
            status = "Not Pressed";
        }

        return status;
    }

    public void openTrapdoor() {
        trapdoorServo.setPosition(1.0);
    }

    public void closeTrapdoor() {
        trapdoorServo.setPosition(0.0);
    }
}
