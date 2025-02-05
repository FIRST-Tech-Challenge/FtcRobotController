package org.firstinspires.ftc.teamcode.opmodes.test.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//

@Autonomous(name="Red Right", group="Simple")
@Disabled
public class RedRight extends LinearOpMode {

    // Constants for motor settings
    private static final double LINEAR_SLIDE_POWER = 0.8;
    private static final double ARM_POWER = 1.0;

    // Positions in degrees (as doubles)
    private static final double INIT_DEGREES = 0.0;
    private static final double GROUND_DEGREES = 5.0;   // Default position (0 degrees)
    private static final double LOW_DEGREES = 15.0;     // Position to pick up from the ground (15 degrees)
    private static final double HIGH_DEGREES = 71.0;    // Position to place into low basket (45 degrees)
    private static final double MAX_DEGREES = 90.0;     // Position to place into an high basket (70 degrees)

    // Formula to calculate ticks per degree
    final double ARM_TICKS_PER_DEGREE =
            145.1 // encoder ticks per rotation of the bare RS-555 motor
                    * 5.2 // gear ratio of the 5.2:1 Yellow Jacket gearbox
                    * 5.0 // external gear reduction, a 20T pinion gear driving a 100T hub-mount gear (5:1 reduction)
                    * 1 / 360.0 *2; // we want ticks per degree, not per rotation
    private final double INIT_POSITION_TICKS = INIT_DEGREES* ARM_TICKS_PER_DEGREE;
    private final double GROUND_POSITION_TICKS = GROUND_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double LOW_POSITION_TICKS = LOW_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double HIGH_POSITION_TICKS = HIGH_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double MAX_POSITION_TICKS = MAX_DEGREES * ARM_TICKS_PER_DEGREE;

    // Declare motor variables
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private  DcMotor armMotor;
    private CRServo intake;


    // Constants
    private static final double POWER = 0.5; // Motor power
    private static double SPEED_HALFPOWER = 2.25;
    private static double SPEED_FULLPOWER = 4.50;

    //left power = 0 moves left, right power = 0 moves right
    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        intake = hardwareMap.get(CRServo.class, "intake");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setArmPosition(INIT_POSITION_TICKS);


        // Set motor directions (reverse right motors for proper movement)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Wait for the game to start
        waitForStart();

        // Move forward for a specified time
        frontLeftMotor.setPower(1.0);
        frontRightMotor.setPower(1.0);
        backLeftMotor.setPower(1.0);
        backRightMotor.setPower(1.0);

        // Sleep for the determined time to move forward 30 inches
        sleep(6670);

        Intake(-1.0,100);

        //Move backward for 20 inches
        frontLeftMotor.setPower(-1.0);
        frontRightMotor.setPower(-1.0);
        backLeftMotor.setPower(-1.0);
        backRightMotor.setPower(-1.0);

        //sleep for the time to move backward 20 inches
        sleep(4440);

        //Turn Right
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(1.0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(1.0);

        sleep(1000);

        //Move forward 50 inches
        frontLeftMotor.setPower(1.0);
        frontRightMotor.setPower(1.0);
        backLeftMotor.setPower(1.0);
        backRightMotor.setPower(1.0);
        sleep(11110);

        //Turn Right
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(1.0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(1.0);

        sleep(1000);

        //Move backward for 20 inches
        frontLeftMotor.setPower(-1.0);
        frontRightMotor.setPower(-1.0);
        backLeftMotor.setPower(-1.0);
        backRightMotor.setPower(-1.0);

        sleep(4440);

        setArmPosition(HIGH_POSITION_TICKS);



        // Stop all motion
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Front Left Power", frontLeftMotor.getPower());
        telemetry.addData("Front Right Power", frontRightMotor.getPower());
        telemetry.addData("Back Left Power", backLeftMotor.getPower());
        telemetry.addData("Back Right Power", backRightMotor.getPower());
        telemetry.addData("Status", "Task Complete");
        telemetry.update();
    }
    private void Intake(double power, long time) {
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
