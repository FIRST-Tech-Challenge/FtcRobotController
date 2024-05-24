package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    DcMotor intakeMotor;

    Servo intakeServo;

    boolean intakeToggle = false;
    boolean intakeDirection = true;

    double intakePower = 1.0;

    public Intake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    // This function controls the intake and conveyor.
    // The first input is the button used to control the trap door.
    // The second input is the time the function uses to space out inputs.
    public void driveIntake(boolean button, ElapsedTime time) {
        if (button && time.time() > .25 && !intakeToggle) {
            intakeToggle = true;
            time.reset();


            intakeMotor.setPower(intakePower);


        }
        else if (button && time.time() > .25 && intakeToggle) {
            intakeToggle = false;
            time.reset();

            intakeMotor.setPower(0.0);

        }
    }


    public void reverseIntake(boolean button, ElapsedTime time) {
        if (button && time.time() > .25 && !intakeDirection) {
            intakeDirection = true;
            time.reset();

            intakePower = 1.0;

        }
        else if (button && time.time() > .25 && intakeToggle) {
            intakeDirection = false;
            time.reset();

            intakePower = -1.0;

        }
    }
    public void turnOffConveyorBelt() {
        intakeMotor.setPower(0.0);
    }

    public void turnOnConveyorBelt() {
        intakeMotor.setPower(1.0);
    }


} // end class




