package org.firstinspires.ftc.teamcode.hardware;

import java.util.HashMap;
import java.util.HashSet;

public class ExtendableArm extends Arm {
    // The motor the arm up and down.
    private final DcMotor ROTATION_MOTOR;
    // The motor that extends and retracts the arm.
    private final DcMotor EXTENSION_MOTOR;

    /*
     * The servos that control the claw.
     * If any of them are not used, 
     * simply set it to null. 
     */
    // The servo that rotates the claw about the X-axis
    private final Servo CLAW_X_SERVO;
    // The servo that rotates the claw about the Y-axis
    private final Servo CLAW_Y_SERVO;
    // The servo that rotates the claw about the Z-axis
    private final Servo CLAW_Z_SERVO;
    // The servo that opens and closes the grip.
    private final Servo CLAW_GRIP_SERVO;

    /**
     * Instantiate the motors using HashMaps.
     * Each key should be the name of the motor as defined by the configuration.
     * Note that the provided keys should be replaced with the current season's.
     * 
     * @param motors A HashMap with motor names as keys and `DcMotor`s as values
     * @param servos A HashMap with servo names as keys and `Servo`s as values
     */
    public ExtendableArm(HashMap<DcMotor> motors, HashMap<Servo> servos) {
        super(motors.values(), servos.values());

        ROTATION_MOTOR = motors.get("rotation_motor");
        EXTENSION_MOTOR = motors.get("extension_motor");

        CLAW_X_SERVO = servos.get("claw_x_servo");
        CLAW_Y_SERVO = servos.get("claw_y_servo");
        CLAW_Z_SERVO = servos.get("claw_z_servo");
        CLAW_GRIP_SERVO = servos.get("claw_grip_servo");
    }

    /**
     * Instantiate each motor individually.
     * 
     * @param rotationMotor  The motor that rotates the arm up and down.
     * @param extensionMotor The motor that extends and retracts the arm.
     * @param clawXServo     The servo that rotates the claw about the X-axis.
     * @param clawYServo     The servo that rotates the claw abou the Y-axis.
     * @param clawZServo     The servo that rotates the claw about the Z-axis.
     * @param clawGripServo  The servo that opens and closes the claw.
     */
    public ExtendableArm(DcMotor rotationMotor, DcMotor extensionMotor, Servo clawXServo, Servo clawYServo,
            Servo clawZServo, Servo clawGripServo) {
        super();
        
        /* Motors */
        this.ROTATION_MOTOR = rotationMotor;
        this.EXTENSION_MOTOR = extensionMotor;

        super.motors.add(rotationMotor);
        super.motors.add(extensionMotor);
        
        /* Servos */
        this.CLAW_X_SERVO = clawXServo;
        this.CLAW_Y_SERVO = clawYServo;
        this.CLAW_Z_SERVO = clawZServo;
        this.CLAW_GRIP_SERVO = clawGripServo;
        
        super.servos.add(clawXServo);
        super.servos.add(clawYServo);
        super.servos.add(clawZServo);
        super.servos.add(clawGripServo);
    }
}