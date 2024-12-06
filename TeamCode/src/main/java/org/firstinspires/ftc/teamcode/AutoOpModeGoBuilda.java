package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name="AutoOpModeGoBuilda", group="Robot")
@Disabled
public class AutoOpModeGoBuilda extends LinearOpMode {

    // Define hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private CRServo intake = null;
    private Servo wrist = null;

    // Constants
    private final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    private final double ARM_COLLAPSED_INTO_ROBOT = 0;
    private final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    private final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
    private final double INTAKE_COLLECT = -1.0;
    private final double INTAKE_OFF = 0.0;
    private final double WRIST_FOLDED_IN = 0.8333;
    private final double WRIST_FOLDED_OUT = 0.25;

    // Encoder ticks per inch for your drivetrain
    private final double TICKS_PER_INCH = 100; // Adjust based on your wheel and encoder specifications
    private final double TICKS_PER_DEGREE_TURN = 10; // Adjust based on your turn calibration

    @Override
    public void runOpMode() {

        // Initialize hardware
        leftDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        armMotor = hardwareMap.get(DcMotor.class, "left_arm");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Step 1: Move arm to collection position and start intake
            moveArmToPosition(ARM_COLLECT);
            intake.setPower(INTAKE_COLLECT);
            wrist.setPosition(WRIST_FOLDED_OUT);
            sleep(1000); // Adjust timing as needed

            // Step 2: Move forward
            driveForwardDistance(24, 0.5); // Drive forward 24 inches at power 0.5
            stopMotors();

            // Step 3: Score sample in low position
            moveArmToPosition(ARM_SCORE_SAMPLE_IN_LOW);
            wrist.setPosition(WRIST_FOLDED_IN);
            intake.setPower(INTAKE_OFF);
            sleep(1000); // Adjust timing as needed

            // Step 4: Turn right
            turnRightAngle(90, 0.5); // Turn right 90 degrees at power 0.5
            stopMotors();

            // Step 5: Collapse arm
            moveArmToPosition(ARM_COLLAPSED_INTO_ROBOT);
            wrist.setPosition(WRIST_FOLDED_IN);
            intake.setPower(INTAKE_OFF);
            sleep(1000); // Adjust timing as needed

            telemetry.addLine("Autonomous sequence complete.");
            telemetry.update();
        }
    }

    private void driveForwardDistance(double inches, double power) {
        int targetTicks = (int) (inches * TICKS_PER_INCH);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(targetTicks);
        rightDrive.setTargetPosition(targetTicks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(power);
        rightDrive.setPower(power);

        while (opModeIsActive() && (leftDrive.isBusy() && rightDrive.isBusy())) {
            telemetry.addData("Target Position", targetTicks);
            telemetry.addData("Current Position", "Left: %d, Right: %d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
            telemetry.update();
        }
    }

    private void turnRightAngle(double degrees, double power) {
        int targetTicks = (int) (degrees * TICKS_PER_DEGREE_TURN);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(targetTicks);
        rightDrive.setTargetPosition(-targetTicks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(power);
        rightDrive.setPower(power);

        while (opModeIsActive() && (leftDrive.isBusy() && rightDrive.isBusy())) {
            telemetry.addData("Turning to Angle", degrees);
            telemetry.addData("Current Position", "Left: %d, Right: %d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
            telemetry.update();
        }
    }

    private void stopMotors() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void moveArmToPosition(double position) {
        armMotor.setTargetPosition((int) position);
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && armMotor.isBusy()) {
            telemetry.addData("Moving arm to position: ", position);
            telemetry.addData("Arm Encoder Position: ", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
