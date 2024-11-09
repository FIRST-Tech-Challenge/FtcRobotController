package org.firstinspires.ftc.teamcode.teleop.newTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.mainEnum;

@TeleOp(name = "Combined Wheel and Arm Control")  // TeleOp mode for combined wheel and arm control
public class finalTeleop extends LinearOpMode {

    // Declare wheel motors for movement
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    // Declare arm motors and servos
    public DcMotor mantis;  // Controls the arm mechanism (mantis)
    public DcMotor lift;    // Lifts the arm up and down
    public DcMotor hopper;  // Moves the hopper for loading/unloading
    public CRServo grabber; // Servo to control grabbing mechanism
    public Servo wrist;     // Controls the wrist position
    public Servo door;      // Controls the "door" that can open/close

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot systems
        initialize();
        setDirection();  // Set motor directions
        setBrakes();  // Set braking behavior for all motors

        waitForStart();  // Wait until the start button is pressed

        // Main control loop - executes during the TeleOp phase
        while (opModeIsActive()) {
            // Wheel control based on gamepad input
            finalMovement();

            // Arm and grabber control based on gamepad input
            finalArm();
            finalGrabber();
        }
    }

    // Initialize all hardware components
    public void initialize() {
        try {
            // Mapping motors to hardware configuration names
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            mantis = hardwareMap.get(DcMotor.class, "mantis");  // Arm motor (mantis)
            lift = hardwareMap.get(DcMotor.class, "lift");      // Lift motor
            hopper = hardwareMap.get(DcMotor.class, "hopper");  // Hopper motor
            grabber = hardwareMap.get(CRServo.class, "grabber"); // Grabber servo
            wrist = hardwareMap.get(Servo.class, "wrist");      // Wrist servo
            door = hardwareMap.get(Servo.class, "door");        // Door servo

            // Send telemetry message to indicate successful initialization
            telemetry.addLine("Initialization complete");
            telemetry.update();
        } catch (Exception e) {
            // Handle errors during initialization and display them on the driver hub
            telemetry.addLine("Initialization error: " + e.getMessage());
            telemetry.update();
        }
    }

    // Set the direction for each motor, reversing as necessary
    public void setDirection() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Reverse some of the arm motors as needed
        lift.setDirection(DcMotor.Direction.REVERSE);
        mantis.setDirection(DcMotor.Direction.REVERSE);
        hopper.setDirection(DcMotor.Direction.REVERSE);
    }

    // Set motors to brake mode when power is set to zero (for precise stopping)
    public void setBrakes() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mantis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Control movement of the robot based on gamepad input for driving
    public void movement(double vertical, double strafe, double turn) {
        // Set motor power for mecanum wheels (allowing for strafing and turning)
        frontLeft.setPower(-vertical - strafe - turn);
        frontRight.setPower(-vertical + strafe + turn);
        backLeft.setPower(-vertical + strafe - turn);
        backRight.setPower(-vertical - strafe + turn);
    }

    // Process gamepad input for driving and apply speed reductions
    public void finalMovement() {
        double reduction = 0.8; // Default driving speed
        double turnReduction = 0.55; // Default turn speed

        // Adjust speed based on button presses for different driving modes
        if (gamepad1.a) {
            reduction = 0.4; // Slow mode
            turnReduction = 0.35;
        } else if (gamepad1.b) {
            reduction = 1;   // Fast mode
            turnReduction = 1;
        } else if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
            reduction = 0.0; // Stop mode
            turnReduction = 0.0;
        }

        // Get joystick inputs for vertical (forward/back), strafe (left/right), and turn
        double vertical = reduction * gamepad1.left_stick_y;
        double turn = -reduction * gamepad1.right_stick_x;
        double strafe = -turnReduction * gamepad1.left_stick_x;

        // Apply movement based on joystick inputs
        movement(vertical, strafe, turn);
    }

    // Control arm movements using the appropriate motor based on the current state
    public void arm(mainEnum state, double speed) {
        switch (state) {
            case LIFT:
                lift.setPower(speed);  // Control lift motor
                break;
            case MANTIS:
                mantis.setPower(speed);  // Control mantis motor
                break;
            case HOPPER:
                hopper.setPower(speed);  // Control hopper motor
                break;
            case STOP:
                // Stop all arm-related motors
                lift.setPower(0);
                mantis.setPower(0);
                hopper.setPower(0);
                break;
        }
    }

    // Process gamepad input for arm control and set appropriate power to motors
    public void finalArm() {
        mainEnum state = mainEnum.MANTIS;  // Default state
        double armSpeed = 0.1;  // Default speed
        double reduction = 0.65;  // Speed reduction factor for the lift

        // Mantis (arm) control using the right stick of gamepad2
        if (gamepad2.right_stick_y > 0) {
            state = mainEnum.MANTIS;
            armSpeed = gamepad2.right_stick_y;
        } else if (gamepad2.right_stick_y < 0) {
            state = mainEnum.MANTIS;
            armSpeed = 0.2 * gamepad2.right_stick_y;
        }

        // Hopper control using the left stick of gamepad2
        if (gamepad2.left_stick_y > 0) {
            state = mainEnum.HOPPER;
            armSpeed = gamepad2.left_stick_y;
        } else if (gamepad2.left_stick_y < 0) {
            state = mainEnum.HOPPER;
            armSpeed = gamepad2.left_stick_y * reduction;
        }

        // Lift control using the triggers of gamepad2
        if (Math.abs(gamepad2.right_trigger) > 0) {
            state = mainEnum.LIFT;
            armSpeed = gamepad2.right_trigger;
        } else if (Math.abs(gamepad2.left_trigger) > 0) {
            state = mainEnum.LIFT;
            armSpeed = -gamepad2.left_trigger;
        }

        // Call the arm function with the determined state and speed
        arm(state, armSpeed);
    }

    // Control grabber, wrist, and door using gamepad2 input
    public void finalGrabber() {
        double collect = 1;    // Collect speed for grabber
        double release = -1;   // Release speed for grabber
        double open = 0;       // Open door position
        double close = 0.3;    // Close door position
        double up = 0;         // Wrist up position
        double down = 0.5;     // Wrist down position

        // Grabber control with buttons X (collect) and Y (release)
        if (gamepad2.x) {
            grabber.setPower(collect);
        } else if (gamepad2.y) {
            grabber.setPower(release);
        } else {
            grabber.setPower(0);  // Stop grabber
        }

        // Door control using D-pad (up to open, down to close)
        if (gamepad2.dpad_up) {
            door.setPosition(open);
        } else if (gamepad2.dpad_down) {
            door.setPosition(close);
        }

        // Wrist control with right bumper (up/down)
        if (gamepad2.right_bumper) {
            wrist.setPosition(up);
        } else if (gamepad2.left_bumper) {
            wrist.setPosition(down);
        }
    }
}

