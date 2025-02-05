package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
//


@Autonomous(name="BlueRight", group="Simple")
@Disabled
public class BlueRight extends LinearOpMode {

    private static double ARM_POWER = 0.8;

    // Declare motor variables
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;
    private Servo wrist;

    // Define constants for wrist servo positions
    private static final double WRIST_COLLECT = 1.0;  // Position for collecting
    private static final double WRIST_DEPOSIT = 0.0; // Position for depositing
    private static final double WRIST_HOME = 0.4;    // Home Position
    // Positions in degrees (as doubles)
    private static final double INIT_DEGREES = 0.0;
    private static final double GROUND_DEGREES = 5.0;   // Default position (0 degrees)
    private static final double LOW_DEGREES = 15.0;     // Position to pick up from the ground (15 degrees)
    private static final double HIGH_DEGREES = 71.0;    // Position to place into low basket (45 degrees)
    private static final double MAX_DEGREES = 95.0;     // Position to place into an high basket (70 degrees)

    // Formula to calculate ticks per degree
    final double ARM_TICKS_PER_DEGREE =
            145.1 // encoder ticks per rotation of the bare RS-555 motor
                    * 5.2 // gear ratio of the 5.2:1 Yellow Jacket gearbox
                    * 5.0 // external gear reduction, a 20T pinion gear driving a 100T hub-mount gear (5:1 reduction)
                    * 1 / 360.0 *2; // we want ticks per degree, not per rotation


    // Pre-calculated arm positions in encoder ticks based on degrees
    private final double INIT_POSITION_TICKS = INIT_DEGREES* ARM_TICKS_PER_DEGREE;
    private final double GROUND_POSITION_TICKS = GROUND_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double LOW_POSITION_TICKS = LOW_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double HIGH_POSITION_TICKS = HIGH_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double MAX_POSITION_TICKS = MAX_DEGREES * ARM_TICKS_PER_DEGREE;



    // Constants
    private static final double POWER = 0.5; // Motor power
    private static double INCHES_PER_SEC = 2.25;

    //left power = 0 moves left, right power = 0 moves right
    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        wrist = hardwareMap.get(Servo.class, "wrist");
        armMotor = hardwareMap.dcMotor.get("armMotor");


        Telemetry();

        // Set motor directions (reverse right motors for proper movement)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setPosition(WRIST_COLLECT);

        // Wait for the game to start
        waitForStart();
        wrist.setPosition(WRIST_HOME);
        setArmPosition(GROUND_POSITION_TICKS);

        // Move forward for a specified time
        frontLeftMotor.setPower(POWER);
        frontRightMotor.setPower(POWER);
        backLeftMotor.setPower(POWER);
        backRightMotor.setPower(POWER);

        // Sleep for the determined time to move forward 10 inches
        sleep(4440);

        // Stop all motion
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

    }

    private void Telemetry() {
        //Initializes telemetry on driver station
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Front Left Power", frontLeftMotor.getPower());
        telemetry.addData("Front Right Power", frontRightMotor.getPower());
        telemetry.addData("Back Left Power", backLeftMotor.getPower());
        telemetry.addData("Back Right Power", backRightMotor.getPower());
        telemetry.update();
    }

    private void setArmPosition(double targetPosition) {
        // Safety check to ensure position is within valid range
        if (targetPosition < GROUND_POSITION_TICKS || targetPosition > (MAX_POSITION_TICKS+5.0)) {
            targetPosition = LOW_POSITION_TICKS; // Set to low/ground position if out of range
        }

        // Convert target position in ticks (double) and set motor
        armMotor.setTargetPosition((int) targetPosition);  // Motor expects integer target position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER);
    }
}
