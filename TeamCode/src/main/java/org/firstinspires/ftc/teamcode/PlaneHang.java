package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PlaneHang {
    Servo droneServo;
    Servo hangServo;
    Servo hangServo2;
    DcMotor hangMotor;
    boolean droneToggle = true;

    boolean hangUp = false;

    public PlaneHang(HardwareMap hMap) {
        droneServo = hMap.get(Servo.class, "droneServo");
        hangMotor = hMap.get(DcMotor.class, "hangMotor");
        hangServo = hMap.get(Servo.class, "hangServo");
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
    }

    // This function controls the hanging servo.
    // The first input is the button used to control the servo.
    // The second input is the time the function uses to space out inputs.
    public void hangServo(boolean button, ElapsedTime time) {
        if (button && time.time() > .50 && !hangUp) {
            hangUp = true;
            time.reset();
            hangServo.setPosition(1.0);
            hangServo2.setPosition(1.0);

        }
        else if (button && time.time() > .50 && hangUp) {
            hangUp = false;
            time.reset();
            hangServo.setPosition(0.0);
            hangServo2.setPosition(0.0);

        }
    }
}

