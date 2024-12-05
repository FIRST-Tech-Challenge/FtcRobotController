package com.kalipsorobotics.actions.intake;

import static java.lang.Math.toRadians;

import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.hardware.Servo;
//0.5 open
//0.15 closed
public class IntakeDoorAction extends KServo {

    private Intake intake;
    private KServo intakeDoorServo;
    private boolean isClosed = true;

    private static final double SERVO_SPEED = 240; // per second
    private static final double SERVO_RANGE = 300;
    public static final boolean FLIP_DIRECTION = false;
    public static final double ZERO_POSITION = 0;

    static public final double INTAKE_DOOR_OPEN_POS = 0.5;
    static public final double INTAKE_DOOR_CLOSE_POS = 0.15;

    private IntakeDoorAction(Servo intakeDoorServo) {
        super(intakeDoorServo, SERVO_SPEED, SERVO_RANGE, ZERO_POSITION, FLIP_DIRECTION);
    }
    public IntakeDoorAction(Intake intake) {
        this(intake.getDoorServo().getServo());
        this.intake = intake;
        this.intakeDoorServo = intake.getDoorServo();
    }

    public void open() {
        intakeDoorServo.setPosition(INTAKE_DOOR_OPEN_POS);
        isClosed = false;
    }

    public void close() {
        intakeDoorServo.setPosition(INTAKE_DOOR_CLOSE_POS);
        isClosed = true;
    }

    public void togglePosition() {
        if (!isClosed) {
            close();
        } else {
            open();
        }
    }

    public KServo getIntakeDoorServo() {
        return intakeDoorServo;
    }

    public Intake getIntake() {
        return intake;
    }
}
