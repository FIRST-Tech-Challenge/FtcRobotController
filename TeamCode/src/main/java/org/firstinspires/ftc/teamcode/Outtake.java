package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    DcMotor stageMotor;
    Servo trapdoorServo;

    boolean trapToggle = true;

    // Initiates the motors and servos we need for this subsystem.
    public Outtake(HardwareMap hwMap) {
        stageMotor = hwMap.get(DcMotor.class, "outtakeMotor");
        stageMotor.setDirection(DcMotor.Direction.REVERSE);
        
        trapdoorServo = hwMap.get(Servo.class, "trapdoorServo");
    }

    // This functions uses one double input to drive the lift.
    public void driveLift(double power) { stageMotor.setPower(power); }

    // This function controls the trapdoor.
    // The first input is the button used to control the trap door.
    // The second input is the time the function uses to space out inputs.
    public void trapdoor(boolean button, ElapsedTime time) {
        if (button && time.time() > .50 && !trapToggle) {
            trapToggle = true;
            time.reset();
            trapdoorServo.setPosition(1.0);

        }
        else if (button && time.time() > .50 && trapToggle) {
            trapToggle = false;
            time.reset();
            trapdoorServo.setPosition(0.0);

        }
    }
}
