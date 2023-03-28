package org.firstinspires.ftc.teamcode.opModes.team2;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
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
 *      Right Joystick Y: Move the Lift.
 *
 *  Grabber:
 *      Triangle: Toggle between open and closed.
 *
 * Hardware:
 *  Drive Train:
 *      Motor 0 and 1 for driving (left and right respectively).
 *
 *  Lift:
 *      Motor 2 for moving lift.
 *
 *   Grabber:
 *      Servo 0 for closing/opening grabber.
 */


@TeleOp(name="USE THIS ONE - Team 2 TeleOp", group="Team 2")
public class Team2TeleOp extends TeleOpLinearModeBase {

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
    // 0 to 1
    public final double SERVO1_OPEN_ANGLE = 0D;
    public double SERVO1_CLOSE_ANGLE = 0.3D;
    public double SERVO2_CLOSE_ANGLE = 0.65D;
    public final double SERVO2_OPEN_ANGLE = 1D;

    private Servo grabberServo1;
    private Servo grabberServo2;
    private boolean grabberOpen = false;

    @Override
    public void run() {
        /* INIT */

        // DRIVE TRAIN
        telemetry = TelemetryContainer.getTelemetry();

        leftMotor = HardwareMapContainer.motor0;
        rightMotor = HardwareMapContainer.motor1;


        // LIFT // TODO: Test
        Motor lift_motor = HardwareMapContainer.motor2;
        // Core Hex Motor has 288 counts/revolution; counts/radian = counts/revn / (radians/revn); 3:1 gear
        lift = new Team2LiftComponent(lift_motor, 0.42, (int)((288 / 3) / (Math.PI*2)), 0);

        // GRABBER // TODO: Test
        telemetry = TelemetryContainer.getTelemetry();

        grabberServo1 = HardwareMapContainer.getServo(0);
        grabberServo2 = HardwareMapContainer.getServo(1);

        new GamepadButton(Inputs.gamepad1, PSButtons.TRIANGLE).whenPressed(() -> {
            if (grabberOpen) {
                grabberServo1.setPosition(SERVO1_CLOSE_ANGLE);
                grabberServo2.setPosition(SERVO2_CLOSE_ANGLE);
                grabberOpen = false;
            } else {
                grabberServo1.setPosition(SERVO1_OPEN_ANGLE);
                grabberServo2.setPosition(SERVO2_OPEN_ANGLE);
                grabberOpen = true;
            }
        });

        new GamepadButton(Inputs.gamepad1, PSButtons.CROSS).whenPressed(() -> {
            runLift(Inputs.gamepad1.getRightY());
        });

//        new GamepadButton(Inputs.gamepad1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> {
//           SERVO1_CLOSE_ANGLE -= 0.05;
//        });
//        new GamepadButton(Inputs.gamepad1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> {
//            SERVO2_CLOSE_ANGLE += 0.05;
//        });

        waitForStart();
        /* START */

        while(opModeIsActive()) {
            super.tick();
            /* TICK */
            runDriveTrain(-Inputs.gamepad1.getLeftY(), Inputs.gamepad1.getLeftX());
            runGrabber();

            // Lift telemetry
            telemetry.addData("[Lift] Last set position to", this.targetPosition);

//            // TEST grabber
//
//            telemetry.addData("GRABBER LEFT CLOSED", SERVO1_CLOSE_ANGLE);
//            telemetry.addData("GRABBER RIGHT CLOSED", SERVO2_CLOSE_ANGLE);

            telemetry.update();
        }
    }

    private void runDriveTrain(double yInput, double xInput) {
        final GamepadEx gamepad = Inputs.gamepad1;

        final double[] inputs = {yInput, -xInput}; //forward speed, turn speed, sideways speed

        //if joystick pos is less than this amount from in the middle, the robot doesn't move.
        final double DEAD_ZONE_SIZE = 0.2D;

        new GamepadButton(gamepad, PSButtons.SQUARE).whenPressed(() -> isSlowMode = !isSlowMode);

        new GamepadButton(gamepad, PSButtons.CIRCLE).whenPressed(() -> isForwardOnlyMode = !isForwardOnlyMode);


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
            inputs[i] = Math.abs(inputs[i]) >= DEAD_ZONE_SIZE ? inputs[i] : 0D;
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
            try {
                lift.setHeight(this.targetPosition);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else {
            telemetry.addLine("[Lift] BUSY");
        }
    }

    private void runGrabber() {
        telemetry.addData("[GRABBER] Current Servo Angle: ", grabberServo1.getPosition());
    }

    private void arcadeDrive(Motor leftMotor, Motor rightMotor, double forwardSpeed, double turnSpeed) {
        leftMotor.set((forwardSpeed - turnSpeed) / 2D);
        rightMotor.set((forwardSpeed + turnSpeed) / -2D); // TODO: not sure if negative needed here or not, please test
    }
}
