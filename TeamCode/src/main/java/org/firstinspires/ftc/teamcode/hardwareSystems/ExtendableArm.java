package org.firstinspires.ftc.teamcode.hardwareSystems;

import java.util.HashSet;

import com.qualcomm.robotcore.hardware.*;

public class ExtendableArm extends Arm {
    /**
     * Passed into the {@code ExtendableArm} constructor.
     * Contains the motors and motor types.
     */
    public static class MotorParams {
        private final HashSet<DcMotor> MOTORS;

        // The motor that rotates the arm up and down.
        private final DcMotor ROTATION_MOTOR;
        // The motor that extends and retracts the arm.
        private final DcMotor EXTENSION_MOTOR;

        // The type of motor used by the arm.
        private final MotorType MOTOR_TYPE;

        public MotorParams(DcMotor rotationMotor, DcMotor extensionMotor) {
            this(rotationMotor, extensionMotor, MotorType.TETRIX_TORQUENADO);
        }

        public MotorParams(DcMotor rotationMotor, DcMotor extensionMotor, MotorType motorType) {
            this.ROTATION_MOTOR = rotationMotor;
            this.EXTENSION_MOTOR = extensionMotor;

            MOTORS = new HashSet<>();
            MOTORS.add(rotationMotor);
            MOTORS.add(extensionMotor);

            this.MOTOR_TYPE = motorType;
        }
    }

    /**
     * Passed into the {@code ExtendableArm} constructor.
     * Contains the servos.
     */
    public static class ServoParams {
        private final HashSet<Servo> SERVOS;

        // The servo that rotates the claw about the X-axis
        private Servo CLAW_X_SERVO;
        // The servo that rotates the claw about the Y-axis
        private Servo CLAW_Y_SERVO;
        // The servo that rotates the claw about the Z-axis
        private Servo CLAW_Z_SERVO;

        public ServoParams(Servo clawXServo, Servo clawYServo, Servo clawZServo) {
            this.CLAW_X_SERVO = clawXServo;
            this.CLAW_Y_SERVO = clawYServo;
            this.CLAW_Z_SERVO = clawZServo;

            SERVOS = new HashSet<>();
            SERVOS.add(clawXServo);
            SERVOS.add(clawYServo);
            SERVOS.add(clawZServo);
        }
    }

    /**
     * Passed into the {@code ExtendableArm} constructor.
     * Contains the min rotation, max rotation, and ticks per degree.
     */
    public static class RotationParams {
        // The minimum rotation of the arm in ticks.
        private final int MIN_ROTATION;
        // The maximum rotation of the arm in ticks.
        private final int MAX_ROTATION;
        ;
        // How many ticks it takes to rotate the arm by one degree.
        private final double TICKS_PER_DEGREE;

        public RotationParams(int minRotation, int maxRotation, double ticksPerDegree) {
            this.MIN_ROTATION = minRotation;
            this.MAX_ROTATION = maxRotation;
            this.TICKS_PER_DEGREE = ticksPerDegree;
        }
    }

    /**
     * Passed into the {@code ExtendableArm} constructor.
     * Contains the min extension and max extension.
     */
    public static class ExtensionParams {
        // The minimum extension of the arm in ticks.
        private final int MIN_EXTENSION;
        // The maximum extension of the arm in ticks.
        private final int MAX_EXTENSION;

        public ExtensionParams(int minExtension, int maxExtension) {
            this.MIN_EXTENSION = minExtension;
            this.MAX_EXTENSION = maxExtension;
        }
    }

    // The motor that rotates the arm up and down.
    private final DcMotor ROTATION_MOTOR;
    // The motor power that the arm uses when rotating.
    private double rotationPower = 1.0;
    // How many ticks it takes to rotate the arm by one degree.
    private final double TICKS_PER_ROTATION_DEGREE;

    // The minimum rotation of the arm in ticks.
    private final int MIN_ROTATION;
    // The maximum rotation of the arm in ticks.
    private final int MAX_ROTATION;

    // The motor that extends and retracts the arm.
    private final DcMotor EXTENSION_MOTOR;
    // The motor power that the arm uses when rotating.
    private double extensionPower = 1.0;
    // The minimum extension of the arm in ticks.
    private final int MIN_EXTENSION;
    // The maximum extension of the arm in ticks.
    private final int MAX_EXTENSION;

    // The servo that rotates the claw about the X-axis
    private final Servo CLAW_X_SERVO;
    // The servo that rotates the claw about the Y-axis
    private final Servo CLAW_Y_SERVO;
    // The servo that rotates the claw about the Z-axis
    private final Servo CLAW_Z_SERVO;
    // How much to gradually move the servo.
    private double servoIncrement = 0.1;

    // The servo that opens and closes the grip.
    private final CRServo INTAKE_SERVO;
    private double intakePower = 0.5;
    private double ejectPower = -1.0;

    /**
     * Instantiates an extendable arm
     * @param motorParams     The motors and motor types.
     * @param servoParams     The servos.
     * @param intakeServo     The intake servo.
     * @param rotationParams  The min rotation, max rotation, and ticks per degree.
     * @param extensionParams The min extension and max extension.
     */
    public ExtendableArm(MotorParams motorParams, ServoParams servoParams, CRServo intakeServo, RotationParams rotationParams, ExtensionParams extensionParams) {
        super(motorParams.MOTORS, servoParams.SERVOS);

        this.ROTATION_MOTOR = motorParams.ROTATION_MOTOR;
        this.TICKS_PER_ROTATION_DEGREE = rotationParams.TICKS_PER_DEGREE;
        this.MIN_ROTATION = rotationParams.MIN_ROTATION;
        this.MAX_ROTATION = rotationParams.MAX_ROTATION;

        this.EXTENSION_MOTOR = motorParams.EXTENSION_MOTOR;
        this.MIN_EXTENSION = extensionParams.MIN_EXTENSION;
        this.MAX_EXTENSION = extensionParams.MAX_EXTENSION;

        this.CLAW_X_SERVO = servoParams.CLAW_X_SERVO;
        this.CLAW_Y_SERVO = servoParams.CLAW_Y_SERVO;
        this.CLAW_Z_SERVO = servoParams.CLAW_Z_SERVO;

        this.INTAKE_SERVO = intakeServo;
    }

    public CRServo getIntakeServo() {
        return INTAKE_SERVO;
    }

    public double getRotationPower() {
        return rotationPower;
    }

    public void setRotationPower(double rotationPower) {
        this.rotationPower = rotationPower;
    }

    public double getExtensionPower() {
        return extensionPower;
    }

    public void setExtensionPower(double extensionPower) {
        this.extensionPower = extensionPower;
    }

    public double getServoIncrement() {
        return servoIncrement;
    }

    public void setServoIncrement(double servoIncrement) {
        this.servoIncrement = servoIncrement;
    }

    public double getIntakePower() {
        return intakePower;
    }

    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    public double getEjectPower() {
        return ejectPower;
    }

    public void setEjectPower(double ejectPower) {
        this.ejectPower = ejectPower;
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
     *
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
     *
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
     *
     * @param direction The direction to rotate the servo in.
     *                  Positive values rotate it clockwise, negative values rotate it counterclockwise.
     */
    public void rotateClawZServo(double direction) {
        double targetPosition = CLAW_Z_SERVO.getPosition()
                + Math.signum(direction) * servoIncrement;
        CLAW_Z_SERVO.setPosition(targetPosition);
    }

    public boolean isIntakeActive() {
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