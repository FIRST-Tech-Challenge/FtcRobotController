package org.firstinspires.ftc.teamcode.hardwareSystems;

import java.util.HashMap;
import java.util.HashSet;

import com.qualcomm.robotcore.hardware.*;

public class ExtendableArm extends Arm {
    // The motor the arm up and down.
    private final DcMotor ROTATION_MOTOR;
    // The motor power that the arm uses when rotating.
    private final double rotationPower = 1.0;
    private final static double TICKS_PER_ROTATION_DEGREE = 1440.0 / 360.0;
    // The maximum rotation of the arm in ticks.
    private final static int MAX_ROTATION = 1000;
    // The minimum rotation of the arm in ticks.
    private final static int MIN_ROTATION = 0;

    // The motor that extends and retracts the arm.
    private final DcMotor EXTENSION_MOTOR;
    // The motor power that the arm uses when rotating.
    private static double extensionPower = 1.0;
    // The maximum extension of the arm in ticks.
    private final static int MAX_EXTENSION = 1000;
    // The minimum extension of the arm in ticks.
    private final static int MIN_EXTENSION = 0;

    // The servo that rotates the claw about the X-axis
    private final Servo CLAW_X_SERVO;
    // The servo that rotates the claw about the Y-axis
    private final Servo CLAW_Y_SERVO;
    // The servo that rotates the claw about the Z-axis
    private final Servo CLAW_Z_SERVO;
    // How much to gradually move the servo.
    private final static double servoIncrement = 0.1;

    // The servo that opens and closes the grip.
    private final CRServo INTAKE_SERVO;
    private static double intakePower = 0.5;
    private static double ejectPower = -1.0;

    /**
     * Instantiate the motors using HashMaps.
     * Each key should be the name of the motor as defined by the configuration.
     * Note that the provided keys should be replaced with the current season's.
     * 
     * @param motors A HashMap with motor names as keys and `DcMotor`s as values.
     * @param servos A HashMap with servo names as keys and `Servo`s as values.
     */
    public ExtendableArm(HashMap<String, DcMotor> motors, HashMap<String, Servo> servos, CRServo intakeServo) {
        super(new HashSet<>(motors.values()), new HashSet<>(servos.values()));

        ROTATION_MOTOR = motors.get("rotationMotor");
        EXTENSION_MOTOR = motors.get("extensionMotor");

        CLAW_X_SERVO = servos.get("clawXServo");
        CLAW_Y_SERVO = servos.get("clawYServo");
        CLAW_Z_SERVO = servos.get("clawZServo");
        INTAKE_SERVO = intakeServo;
    }

    /**
     * Instantiate each motor and servo individually.
     * 
     * @param rotationMotor  The motor that rotates the arm up and down.
     * @param extensionMotor The motor that extends and retracts the arm.
     * @param clawXServo     The servo that rotates the claw about the X-axis.
     * @param clawYServo     The servo that rotates the claw about the Y-axis.
     * @param clawZServo     The servo that rotates the claw about the Z-axis.
     * @param intakeServo    The servo that opens and closes the claw.
     */
    public ExtendableArm(DcMotor rotationMotor, DcMotor extensionMotor, Servo clawXServo, Servo clawYServo,
            Servo clawZServo, CRServo intakeServo) {
        super();
        
        /* Motors */
        this.ROTATION_MOTOR = rotationMotor;
        this.EXTENSION_MOTOR = extensionMotor;

        super.MOTORS.add(rotationMotor);
        super.MOTORS.add(extensionMotor);
        
        /* Servos */
        this.CLAW_X_SERVO = clawXServo;
        this.CLAW_Y_SERVO = clawYServo;
        this.CLAW_Z_SERVO = clawZServo;
        this.INTAKE_SERVO = intakeServo;
        
        super.SERVOS.add(clawXServo);
        super.SERVOS.add(clawYServo);
        super.SERVOS.add(clawZServo);
    }

    public CRServo getIntakeServo() {
        return INTAKE_SERVO;
    }

    /**
     * Rotate the arm with a set velocity.
     * Stop the motor if it is out of bounds.
     *
     * @param direction The direction that the arm should rotate in.
     *                  Positive rotates it up, negative rotates it down, zero stops the motor.
     */
    public void rotateArm(double direction) {
        if (ROTATION_MOTOR.getCurrentPosition() > MAX_ROTATION || ROTATION_MOTOR.getCurrentPosition() < MIN_ROTATION) {
            ROTATION_MOTOR.setPower(0);
            return;
        }

        ROTATION_MOTOR.setPower(Math.signum(direction) * rotationPower);
    }

    /**
     * Rotates the arm to a position specified in degrees.
     *
     * @param degrees The position the arm moves to.
     *                The arm's starting position is 0 degrees.
     */
    public void rotateArmToPosition(double degrees) {
        int targetPosition = (int) Math.round(degrees * TICKS_PER_ROTATION_DEGREE);

        // keep the target position within acceptable bounds
        targetPosition = Math.min(Math.max(targetPosition, MIN_ROTATION), MAX_ROTATION);

        /*
         * Calculate the direction that the arm will have to rotate.
         * Negative is down, positive is up
         */
        int direction = (int) Math.signum(targetPosition - ROTATION_MOTOR.getCurrentPosition());

        ROTATION_MOTOR.setTargetPosition(targetPosition);
        ROTATION_MOTOR.setPower(direction * rotationPower);
        ROTATION_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // adjust the extension of the arm to keep the arm length constant
        EXTENSION_MOTOR.setTargetPosition(targetPosition / -1);
        EXTENSION_MOTOR.setPower(0.4);
        EXTENSION_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     *
     * @param direction The direction that the extension motor moves.
     *                  Positive values extend the arm, negative values retract it.
     */
    public void extendArm(double direction) {
        if (EXTENSION_MOTOR.getCurrentPosition() > MAX_EXTENSION || EXTENSION_MOTOR.getCurrentPosition() < MIN_EXTENSION) {
            EXTENSION_MOTOR.setPower(0);
            return;
        }

        EXTENSION_MOTOR.setPower(Math.signum(direction) * extensionPower);
    }

    public void extendArmToPosition(int targetPosition) {
        EXTENSION_MOTOR.setTargetPosition(targetPosition);
        int direction = (int) Math.signum(targetPosition - EXTENSION_MOTOR.getCurrentPosition());
        EXTENSION_MOTOR.setPower(direction * extensionPower);
        EXTENSION_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Rotate the X rotation servo.
     * @param direction The direction to rotate the servo in.
     *                  Positive values rotate it clockwise, negative values rotate it counterclockwise.
     */
    public void rotateClawXServo(double direction) {
        double targetPosition = CLAW_X_SERVO.getPosition()
                                + Math.signum(direction) * servoIncrement;
        CLAW_X_SERVO.setPosition(targetPosition);
    }

    /**
     * Rotate the Y rotation servo.
     * @param direction The direction to rotate the servo in.
     *                  Positive values rotate it clockwise, negative values rotate it counterclockwise.
     */
    public void rotateClawYServo(double direction) {
        double targetPosition = CLAW_Z_SERVO.getPosition()
                                + Math.signum(direction) * servoIncrement;
        CLAW_Y_SERVO.setPosition(targetPosition);
    }

    /**
     * Rotate the Z rotation servo.
     * @param direction The direction to rotate the servo in.
     *                  Positive values rotate it clockwise, negative values rotate it counterclockwise.
     */
    public void rotateClawZServo(double direction) {
        double targetPosition = CLAW_Z_SERVO.getPosition()
                                + Math.signum(direction) * servoIncrement;
        CLAW_Z_SERVO.setPosition(targetPosition);
    }

    public boolean intakeActive() {
        return INTAKE_SERVO.getPower() != 0;
    }

    public void startIntake() {
        INTAKE_SERVO.setPower(intakePower);
    }

    public void stopIntake() {
        INTAKE_SERVO.setPower(0);
    }

    /**
     * Make the intake spin in reverse and eject the object.
     */
    public void ejectIntake() {
        INTAKE_SERVO.setPower(ejectPower);
    }
}