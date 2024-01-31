package org.firstinspires.ftc.teamcode.kuykendall;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.shared.MotionHardwareG2;

@TeleOp(name="BeastOp1", group="Test")
public class BeastOp1 extends OpMode {

    // Servos from intake1
    private Servo wristServo;
    private CRServo intServo;
    private Servo bucketServo;

    // Servos from IntDelivery
    private Servo rightInt, leftInt;

    // Positions for IntDelivery servos
    private final double pickupPosition = .65;
    private final double dropoffPosition = .2;
    private final double outSlide = .33;
    private final double inSlide = .82;
    // MOtors and stuff
    private DcMotor armMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor backLeftMotor;
    public static double driveScale = 1; //was .3
    public static double strafeScale = 1; // was .5
    public static double rotateScale = .6; // was .3

    // State variables
    private boolean intServoState = false;
    private boolean intServoReverse = false;

    // Sequence control variables
    private int sequenceStep = 0;
    private long stepStartTime;
    private final long stepDuration = 500; // Step duration in milliseconds
    private boolean slowmoActive = false;
    private boolean slowmoToggle = false; // To track the toggle state



    @Override
    public void init() {
        // Initialize servos from intake1
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        intServo = hardwareMap.get(CRServo.class, "intServo");
        wristServo.setPosition(0.8); // Initial wrist position

        // Initialize servos from IntDelivery
        rightInt = hardwareMap.get(Servo.class, "rightInt");
        leftInt = hardwareMap.get(Servo.class, "leftInt");
        rightInt.setPosition(pickupPosition);
        leftInt.setPosition(1.0 - pickupPosition);

        //motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Arm initialization
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize bucketServo
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        // You might want to set an initial position for bucketServo here
    }

    @Override
    public void loop() {
        // Introduce separate scales for forward and backward driving
        double forwardDriveScale = 1; // Set this to your desired scale for forward motion (was.5)
        double backwardDriveScale = 1; // Set this to your desired scale for backward motion (was.3)
        double strafe = -gamepad2.left_stick_x * strafeScale;
        double rotate = -gamepad2.right_stick_x * rotateScale;

        // Toggle slowmo on dpad_up press
        if (gamepad2.left_stick_button && !slowmoToggle) {
            slowmoActive = !slowmoActive;
            slowmoToggle = true;
        } else if (!gamepad2.left_stick_button) {
            slowmoToggle = false;
        }

        double speedModifier = slowmoActive ? 0.25 : 1.0; // Reduce speed to 1/4 if slowmo is active
        // Drivetrain logic
        double rawDrive = gamepad2.left_stick_y;
        double drive;
        if (rawDrive > 0) { // Forward
            drive = rawDrive * forwardDriveScale;
        } else { // Backward or no motion
            drive = rawDrive * backwardDriveScale;
        }

        double frontLeftPower = (drive + strafe + rotate) * speedModifier;
        double frontRightPower = (drive - strafe - rotate) * speedModifier;
        double backLeftPower = (drive - strafe + rotate) * speedModifier;
        double backRightPower = (drive + strafe - rotate) * speedModifier;


        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
        // Check the current step in the sequence
        switch(sequenceStep) {
            case 0:
                // Normal operation, waiting for Y button press
                if (gamepad1.y) {
                    // Move wrist first
                    wristServo.setPosition(.8); // Adjust to your wrist up position

                    // Move to next step and record time
                    sequenceStep = 1;
                    stepStartTime = System.currentTimeMillis();
                }
                break;

            case 1:
                // Check if delay has passed
                if (System.currentTimeMillis() - stepStartTime >= stepDuration) {
                    // Move delivery to up position
                    rightInt.setPosition(pickupPosition);
                    leftInt.setPosition(1.0 - pickupPosition);

                    // Reset sequence step to 0 for normal operation
                    sequenceStep = 0;
                }
                break;

            // ... [Implement other steps as needed]

            default:
                // Handle default case
                break;
        }

        // New function to move delivery down then wrist down
        if (gamepad1.a) {
            rightInt.setPosition(dropoffPosition);
            leftInt.setPosition(1.0 - dropoffPosition);
            wristServo.setPosition(0.4); // Wrist down position
        }
        // Toggle continuous servo on 'x' press
        if (gamepad1.x) {
            intServoState = !intServoState;
            if (intServoState) {
                intServo.setPower(2.0); // Spin forward
            } else {
                intServo.setPower(0); // Stop spinning
            }
        }

        // Spin continuous servo forward while 'X' is pressed
        if (gamepad1.x) {
            intServo.setPower(2.0); // Spin forward
        }
        // Spin continuous servo backward while 'B' is pressed
        else if (gamepad1.b) {
            intServo.setPower(-2.0); // Spin backward
        }
        // Stop the servo when neither X nor B is pressed
        else {
            intServo.setPower(0); // Stop spinning
        }

        if (gamepad1.right_bumper) {
            bucketServo.setPosition(inSlide);
        }
        if (gamepad1.left_bumper) {
            bucketServo.setPosition(outSlide);
        }
        // Resetting the armMotor Encoder
        if (gamepad1.dpad_left) {
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        // Add telemetry data to debug
        telemetry.addData("Wrist Servo Position", wristServo.getPosition());
        telemetry.addData("Right Intake Servo Position", rightInt.getPosition());
        telemetry.addData("Left Intake Servo Position", leftInt.getPosition());
        telemetry.update();
    }
}
