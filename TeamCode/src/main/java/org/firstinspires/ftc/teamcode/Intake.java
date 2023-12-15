package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    DcMotor intakeMotor, conveyorMotor;
    Servo intakeServo;

    boolean intakeToggle = false;

    public Intake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        conveyorMotor = hwMap.get(DcMotor.class, "conveyorMotor");
        conveyorMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeServo = hwMap.get(Servo.class, "intakeServo");
    }

    // This function controls the intake and conveyor.
    // The first input is the button used to control the trap door.
    // The second input is the time the function uses to space out inputs.
    public void driveIntake(boolean button, ElapsedTime time) {
        if (button && time.time() > .25 && !intakeToggle) {
            intakeToggle = true;
            time.reset();

            intakeMotor.setPower(1.0);
            conveyorMotor.setPower(1.0);

        }
        else if (button && time.time() > .25 && intakeToggle) {
            intakeToggle = false;
            time.reset();

            intakeMotor.setPower(0.0);
            conveyorMotor.setPower(0.0);

        }
    }

    public void intakeDown(boolean down) {
        if (down) {
            intakeServo.setPosition(1.0);
        }
        else {
            intakeServo.setPosition(0.0);
        }
    }
}
