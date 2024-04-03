package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PlaneHang {
    Servo droneServo, leftHangServo, rightHangServo;
    DcMotor hangMotor;
    boolean droneToggle = true;

    boolean hangUp = false;

    public PlaneHang(HardwareMap hMap) {
        droneServo = hMap.get(Servo.class, "droneServo");

        hangMotor = hMap.get(DcMotor.class, "hangMotor");
        hangMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftHangServo = hMap.get(Servo.class, "leftHangServo");
        rightHangServo = hMap.get(Servo.class, "rightHangServo");

    }

    // This function controls the drone.
    // The first input is the button used to control the drone.
    // The second input is the time the function uses to space out inputs.
    public void launchDrone(boolean button, ElapsedTime time) {
        if (button && time.time() > .75 && !droneToggle) {
            droneToggle = true;
            time.reset();
            droneServo.setPosition(0.0);

        } else if (button && time.time() > .75 && droneToggle) {
            droneToggle = false;
            time.reset();
            droneServo.setPosition(1.0);
        }


    }

    // This function controls the hanging motors.
    // The first input is the button used to power the motors.
    // The second input is the button used to reverse the motors.
    public void hangMotors(boolean button, boolean button2 ) {
        if (button) {
            hangMotor.setPower(1.0);

        } else if (button2) {
            hangMotor.setPower(-1.0);

        }
        else {
            hangMotor.setPower(0.0);
        }
    }

    // This function controls the hanging servo.
    // The first input is the button used to control the servo.
    // The second input is the time the function uses to space out inputs.
    public void hangServo(boolean button, ElapsedTime time) {
        if (button && time.time() > .50 && !hangUp) {
            hangUp = true;
            time.reset();
            leftHangServo.setPosition(0.1);
            rightHangServo.setPosition(1);

        }
        else if (button && time.time() > .50 && hangUp) {
            hangUp = false;
            time.reset();
            leftHangServo.setPosition(0.6);
            rightHangServo.setPosition(0.5);

        }
    }

    public void initServo() {
        leftHangServo.setPosition(0.6);
        rightHangServo.setPosition(0.5);
    }
}

