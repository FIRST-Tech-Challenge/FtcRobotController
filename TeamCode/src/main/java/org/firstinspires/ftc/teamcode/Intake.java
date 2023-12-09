package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    DcMotor intakeMotor, conveyorMotor;
    Servo intakeServo;
    public Intake(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        conveyorMotor = hwMap.get(DcMotor.class, "conveyorMotor");
        conveyorMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeServo = hwMap.get(Servo.class, "intakeServo");
    }

    public void driveIntake(boolean powered) {
        if (powered) {
            intakeMotor.setPower(1.0);
            conveyorMotor.setPower(1.0);
        }
        else {
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
