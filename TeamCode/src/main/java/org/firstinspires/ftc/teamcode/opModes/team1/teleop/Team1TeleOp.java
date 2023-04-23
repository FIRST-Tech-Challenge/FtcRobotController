
package org.firstinspires.ftc.teamcode.opModes.team1.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

import org.firstinspires.ftc.teamcode.components.Team2LiftComponent;


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
 *      Cross: Toggle between extended and not.
 *
 *  Grabber:
 *      Triangle: Toggle between open and closed.
 *
 * Hardware:
 *  Drive Train:
 *      Motor 0 and 1 for driving (left and right respectively)
 *
 *  Lift:
 *      Motor 2 for moving lift.
 *
 *   Grabber:
 *      Servo 0 for closing/opening grabber.
 */


@TeleOp(name="Team 1 TeleOp", group="Team 1")
public class Team1TeleOp extends TeleOpModeBase {

    private Telemetry telemetry;

    // DRIVE TRAIN
    private Motor leftMotor;
    private Motor rightMotor;

    private boolean isSlowMode = false;
    private boolean isForwardOnlyMode = false;

    // LIFT
    private Motor liftMotor;
    private boolean motorPosition; // true if going to 120 or on 120, false if going to 240 or in 240

    // GRABBER
    // 0 to 1
    public final double SERVO_OPEN_ANGLE = 0D;
    public final double SERVO_CLOSE_ANGLE = 0.25D;

    private double grabberAngle = SERVO_OPEN_ANGLE;

    private Servo grabberServo;

    @Override
    public void setup() {
        // DRIVE TRAIN
        telemetry = TelemetryContainer.getTelemetry();

        leftMotor = HardwareMapContainer.motor0;
        rightMotor = HardwareMapContainer.motor1;


        // LIFT
        liftMotor = HardwareMapContainer.motor2;
        liftMotor.setRunMode(Motor.RunMode.PositionControl);
        liftMotor.setDistancePerPulse(0.015);
        new GamepadButton(
                Inputs.gamepad1, PSButtons.CROSS)
                .whenPressed(new InstantCommand(() -> {
            if (motorPosition){
                liftMotor.setTargetDistance(18.0);
            }
            else{
                liftMotor.setTargetDistance(0);
            }
            motorPosition = !motorPosition;
        }));

        // GRABBER
        telemetry = TelemetryContainer.getTelemetry();

        telemetry.addData("[GRABBER] Open Angle: ", SERVO_OPEN_ANGLE);
        telemetry.addData("[GRABBER] Close Angle: ", SERVO_CLOSE_ANGLE);

        grabberServo = null;//HardwareMapContainer.getServo(0);

        new GamepadButton(Inputs.gamepad1, PSButtons.TRIANGLE).whenActive(() -> {
            if (grabberAngle == SERVO_OPEN_ANGLE) {
                grabberServo.setPosition(SERVO_OPEN_ANGLE);
                grabberAngle = SERVO_OPEN_ANGLE;
            } else {
                grabberServo.setPosition(SERVO_CLOSE_ANGLE);
                grabberAngle = SERVO_CLOSE_ANGLE;
            }
        });
    }

    @Override
    public void every_tick() {
        runDriveTrain(Inputs.gamepad1.getLeftY(), Inputs.gamepad1.getRightY());
        runLift();
        runGrabber();
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

    private void runLift() {
        liftMotor.set(0.5);

        if (motorPosition) {
            telemetry.addLine("[Lift] Motor at/going to 120");
        } else {
            telemetry.addLine("[Lift] Motor at/going to 240");
        }
    }

    private void runGrabber() {
        telemetry.addData("[GRABBER] Current Servo Angle: ", grabberServo.getPosition());
    }

    private void arcadeDrive(Motor leftMotor, Motor rightMotor, double forwardSpeed, double turnSpeed) {
        leftMotor.set((forwardSpeed - turnSpeed) / 2D);
        rightMotor.set((forwardSpeed + turnSpeed) / -2D); // TODO: not sure if negative needed here or not, please test
    }
}
