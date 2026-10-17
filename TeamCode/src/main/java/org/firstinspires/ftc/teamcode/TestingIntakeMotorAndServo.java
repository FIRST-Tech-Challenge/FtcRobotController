package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Motor and Servo Testing", group="Linear OpMode")
public class TestingIntakeMotorAndServo extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intakeMotor = null;
    private Servo intakeServo = null;
    private DcMotor elevatorMotor = null;

    // Motor States
    private boolean motorIsSpinning = false;
    private boolean lastButtonState = false;

    // Servo States
    private boolean servoMoved = false;
    private boolean servoLastButtonState = false;

    @Override
    public void runOpMode() {
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        elevatorMotor = hardwareMap.get(DcMotor.class, "ElevatorMotor");

        intakeServo.setDirection(Servo.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // --- Motor Toggle Control (X Button) ---
            boolean currentButtonState = gamepad1.x;

            if (currentButtonState && !lastButtonState) {
                motorIsSpinning = !motorIsSpinning;
            }
            lastButtonState = currentButtonState;

            if (motorIsSpinning) {
                intakeMotor.setPower(-1.0);
                elevatorMotor.setPower(RobotConfig.ELEVATOR_POWER);
            } else {
                intakeMotor.setPower(0.0);
                elevatorMotor.setPower(0.0);
            }

            // --- Servo Toggle Control (Y Button) ---
            boolean servoCurrentButtonState = gamepad1.y;

            // Only triggers on the exact frame the button is pressed (Rising Edge)
            if (servoCurrentButtonState && !servoLastButtonState) {
                servoMoved = !servoMoved; // Flip tracking state

                if (servoMoved) {
                    intakeServo.setPosition(0.5); // Adjusted to 50% travel for a visible movement test
                } else {
                    intakeServo.setPosition(0.0); // Reset to 0% travel
                }
            }
            // Updates button history for the next frame loop
            servoLastButtonState = servoCurrentButtonState;

            // --- Telemetry ---
            telemetry.addData("Servo Position", intakeServo.getPosition());
            telemetry.addData("Motor Spinning", motorIsSpinning);
            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("Elevator Power", RobotConfig.ELEVATOR_POWER);
            telemetry.update();
        }
    }
}
