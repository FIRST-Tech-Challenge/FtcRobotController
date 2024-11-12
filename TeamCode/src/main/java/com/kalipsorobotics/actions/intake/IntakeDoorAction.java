package com.kalipsorobotics.actions.intake;

import static java.lang.Math.toRadians;

import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.hardware.Servo;
//0.5 open
//0.15 closed
public class IntakeDoorAction extends KServo {

    private Intake intake;
    private Servo intakeDoorServo;
    private boolean isClosed = true;

    private static final double SERVO_SPEED = 240; // per second
    private static final double SERVO_RANGE = 300;
    public static final boolean FLIP_DIRECTION = false;
    public static final double ZERO_POSITION = 45;

    private IntakeDoorAction(Servo intakeDoorServo) {
        super(intakeDoorServo, SERVO_SPEED, SERVO_RANGE, ZERO_POSITION, FLIP_DIRECTION);
    }
    public IntakeDoorAction(Intake intake) {
        this(intake.getDoorServo());
        this.intake = intake;
        this.intakeDoorServo = intake.getDoorServo();
    }

    public void open() {
        intakeDoorServo.setPosition(0.5);
        isClosed = false;
    }

    public void close() {
        intakeDoorServo.setPosition(0.15);
        isClosed = true;
    }

    public void togglePosition() {
        if (!isClosed) {
            close();
        } else {
            open();
        }
    }

    public Servo getIntakeDoorServo() {
        return intakeDoorServo;
    }

    public Intake getIntake() {
        return intake;
    }
}
