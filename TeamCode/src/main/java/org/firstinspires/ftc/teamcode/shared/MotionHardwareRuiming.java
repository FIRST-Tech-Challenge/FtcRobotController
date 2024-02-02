package org.firstinspires.ftc.teamcode.shared;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.function.BooleanSupplier;

public class MotionHardwareRuiming {
    // Hardware components
    public DcMotor armMotor;
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public Servo wristServo;
    public CRServo intServo;
    public Servo bucketServo;
    public Servo rightInt;
    public Servo leftInt;

    // Variables for sequence control
    public int sequenceStep = 0;
    public long stepStartTime;
    public final long stepDuration = 1000; // Duration for each step, in milliseconds

    // Constants for arm position limits
    private static final double MAX_EXTENSION_INCHES = 5.0;
    private static final double MIN_EXTENSION_INCHES = 0.0;

    // Encoder Constants
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double SPOOL_DIAMETER_MM = 33.401;
    static final double MM_PER_INCH = 25.4;
    static final double SPOOL_DIAMETER_INCHES = SPOOL_DIAMETER_MM / MM_PER_INCH;
    static final double ARM_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (SPOOL_DIAMETER_INCHES * Math.PI);

    // Constructor
    public MotionHardwareRuiming() {
    }

    // Initialize hardware
    public void init(HardwareMap hardwareMap) {
        // Motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        // Setting motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        // Setting the mode of motors
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Servos
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        intServo = hardwareMap.get(CRServo.class, "intServo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        rightInt = hardwareMap.get(Servo.class, "rightInt");
        leftInt = hardwareMap.get(Servo.class, "leftInt");

        // Initial positions for servos (adjust as needed)
        wristServo.setPosition(0.8); // Initial position
        rightInt.setPosition(0.65); // Pickup position
        leftInt.setPosition(0.35); // Pickup position
    }

    // setArmPosition method
    public void setArmPosition(double speed, double positionInches, double timeoutS, ElapsedTime runtime, Telemetry telemetry, BooleanSupplier opModeIsActive) {
        int newArmTarget; // Ensure that this variable is declared within the method

        // Ensure position is within limits
        positionInches = Math.max(MIN_EXTENSION_INCHES, Math.min(MAX_EXTENSION_INCHES, positionInches));

        // Ensure that the opmode is still active
        if (opModeIsActive.getAsBoolean()) {
            // Calculate the new target position in encoder counts
            newArmTarget = (int)(positionInches * ARM_COUNTS_PER_INCH);

            // Set the target position and switch the motor to RUN_TO_POSITION mode
            armMotor.setTargetPosition(newArmTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start motion by setting the motor speed
            runtime.reset();
            armMotor.setPower(Math.abs(speed));

            // Continuously monitor the arm's progress in the loop
            while (opModeIsActive.getAsBoolean() && (runtime.seconds() < timeoutS) && (armMotor.isBusy())) {
                // Add telemetry data for monitoring
                telemetry.addData("Target Position (inches): ", positionInches);
                telemetry.addData("Encoder Target", newArmTarget);
                telemetry.addData("Current Arm Position", armMotor.getCurrentPosition());
                telemetry.update();
            }

            // Once the arm has reached its position or the timeout has expired, stop the motor
            armMotor.setPower(0);

            // Reset the motor to RUN_USING_ENCODER mode for regular operation
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
