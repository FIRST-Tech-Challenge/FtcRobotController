package org.firstinspires.ftc.teamcode.Utilities.Templates;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SubsystemTemplate {
    private ElapsedTime runtime;

    //Declare Variables
    private DcMotor exampleMotor;
    private Servo exampleServo;

    //Constructor intakes hardwareMaps and runtime.
    public SubsystemTemplate(DcMotor exampleMotor, Servo exampleServo, ElapsedTime runtime) {
        this.exampleMotor = exampleMotor;
        this.exampleServo = exampleServo;
        this.runtime = runtime;

        initialize();
    }

    //Initializes Hardware
    private void initialize() {
        exampleMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        exampleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        exampleServo.setDirection(Servo.Direction.FORWARD);
        exampleServo.scaleRange(0.2, 0.8);
    }

    //Subsystem Actions Below
    public void exampleActionOne(double testParam) {
        exampleMotor.setPower(testParam);
        exampleServo.setPosition(0.4);
    }

    public void exampleActionTwo() {
        exampleMotor.setPower(0.0);
        exampleServo.setPosition(0.0);
    }

    public void exampleActionThree() {
        exampleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
