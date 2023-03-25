package org.firstinspires.ftc.teamcode.opModes.team2;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

import org.firstinspires.ftc.teamcode.components.Team2LiftComponent;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;


/**
 * Team 2 TeleOp
 *
 * Controls:
 *  Drive Train:
 *      Left Joystick: Control the drive train in a normal way.
 *      Square Toggle between normal speed and slow speed.
 *      Circle: Toggle between normal drive and forward only (no turning).
 *
 *  Lift:
 *      Right Joystick Y: Move the Lift.
 *
 * Hardware:
 *  Drive Train:
 *      Motor 0 and 1 for driving (left and right respectively)
 *
 *  Lift:
 *      Motor 3 for moving lift
 */


@TeleOp(name="Team 2 TeleOp", group="Team 2")
public class Team2TeleOp extends TeleOpModeBase {

    private Telemetry telemetry;

    // DRIVE TRAIN
    private Motor leftMotor;
    private Motor rightMotor;

    private boolean isSlowMode = false;
    private boolean isForwardOnlyMode = false;

    // LIFT
    private Team2LiftComponent lift;
    private double targetPosition;

    // GRABBER
    public final double SERVO_OPEN_ANGLE = 0D;
    public final double SERVO_CLOSE_ANGLE = 90D;

    private ServoEx grabberServo;

    @Override
    public void setup() {
        // DRIVE TRAIN
        telemetry = TelemetryContainer.getTelemetry();

        leftMotor = HardwareMapContainer.motor0;
        rightMotor = HardwareMapContainer.motor1;


        // LIFT
        Motor lift_motor = HardwareMapContainer.motor3;
        // Core Hex Motor has 288 counts/revolution; counts/radian = counts/revn / (radians/revn); 3:1 gear
        lift = new Team2LiftComponent(lift_motor, 0.42, (int)((288 / 3) / (Math.PI*2)), 0);

        // GRABBER
        telemetry = TelemetryContainer.getTelemetry();

        telemetry.addData("[GRABBER] Open Angle: ", SERVO_OPEN_ANGLE);
        telemetry.addData("[GRABBER] Close Angle: ", SERVO_CLOSE_ANGLE);

        grabberServo = new SimpleServo(HardwareMapContainer.getMap(), "grabberServo",
                SERVO_OPEN_ANGLE, SERVO_CLOSE_ANGLE);

        new GamepadButton(Inputs.gamepad1, PSButtons.CIRCLE).whenActive(() ->
                grabberServo.setPosition(SERVO_CLOSE_ANGLE)
        );

        new GamepadButton(Inputs.gamepad1, PSButtons.SQUARE).whenActive(() ->
                grabberServo.setPosition(SERVO_OPEN_ANGLE)
        );
    }

    @Override
    public void every_tick() {
        runDriveTrain(Inputs.gamepad1.getLeftY(), Inputs.gamepad1.getRightY());
        runLift(Inputs.gamepad1.getRightY());
    }

    private void runDriveTrain(double yInput, double xInput) {
        final GamepadEx gamepad = Inputs.gamepad1;

        final double[] inputs = {yInput, xInput}; //forward speed, turn speed, sideways speed

        //if joystick pos is less than this amount from in the middle, the robot doesn't move.
        final double DEAD_ZONE_SIZE = 0.2D;

        new GamepadButton(gamepad, PSButtons.SQUARE).whenActive(() -> isSlowMode = !isSlowMode);

        new GamepadButton(gamepad, PSButtons.CIRCLE).whenActive(() -> isForwardOnlyMode = !isForwardOnlyMode);


        if (isSlowMode) {
            for (int i = 0; i < inputs.length; i++) {
                inputs[i] /= 2D;
            }
        }

        if (isForwardOnlyMode) {
            inputs[1] = 0D; //set turn speed to 0
        }

        //if the following variables are less than DEAD_ZONE_SIZE from 0, set them to be 0
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = Math.abs(inputs[i]) < DEAD_ZONE_SIZE ? inputs[i] : 0D;
        }

        arcadeDrive(leftMotor, rightMotor, inputs[0], inputs[1]);

        telemetry.addData("[Drive Train] Y", inputs[0]);
        telemetry.addData("[Drive Train] X", inputs[1]);

        telemetry.addData("[Drive Train] Dead zone size", DEAD_ZONE_SIZE);

        if (isSlowMode) {
            telemetry.addLine("[Drive Train] Slow mode active");
        }

        if (isForwardOnlyMode) {
            telemetry.addLine("[Drive Train] Forward only mode active");
        }
    }

    private void runLift(double inputPos) {
        if(Math.abs(inputPos) != this.targetPosition) {
            this.targetPosition = Math.abs(inputPos);
            lift.setHeight(this.targetPosition);
        } else {
            telemetry.addLine("[Lift] BUSY");
        }
        telemetry.addData("[Lift] Last set position to", this.targetPosition);
        telemetry.addData("[Lift] Position of gamepad", Math.abs(inputPos));
    }

    private void runGrabber() {
        telemetry.addData("[GRABBER] Current Servo Angle: ", grabberServo.getAngle());
    }

    private void arcadeDrive(Motor leftMotor, Motor rightMotor, double forwardSpeed, double turnSpeed) {
        leftMotor.set((forwardSpeed - turnSpeed) / 2D);
        rightMotor.set((forwardSpeed + turnSpeed) / -2D); // TODO: not sure if negative needed here or not, please test
    }
}
