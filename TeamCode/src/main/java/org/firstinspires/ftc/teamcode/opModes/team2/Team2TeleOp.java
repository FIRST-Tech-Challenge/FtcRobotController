package org.firstinspires.ftc.teamcode.opModes.team2;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
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
 * A program that controls a drive train has special wheels to go sideways.
 * Controls:
 *
 *  Left Joystick: Control the drive train in a normal way.
 *  Right Joystick X: Make drive train go sideways.
 *
 *  Square Toggle between normal speed and slow speed.
 *  Circle: Toggle between both directions and forward only.
 */

@Disabled
@TeleOp(name="Team2TeleOp", group="Team 2")
public class Team2TeleOp extends TeleOpModeBase {

    private Telemetry telemetry;

    Team2LiftComponent lift;

    double targetPosition;

    private Motor leftMotor;
    private Motor rightMotor;
    private Motor sidewaysMotor;

    private boolean isSlowMode = false;
    private boolean isForwardOnlyMode = false;

    @Override
    public void setup() {
        telemetry = TelemetryContainer.getTelemetry();

        // Motors used to travel
        leftMotor = HardwareMapContainer.motor0;
        rightMotor = HardwareMapContainer.motor1;
        sidewaysMotor = HardwareMapContainer.motor2;

        // Lift Motor
        Motor lift_motor = HardwareMapContainer.motor3;
        // Core Hex Motor has 288 counts/revolution; counts/radian = counts/revn / (radians/revn); 3:1 gear
        lift = new Team2LiftComponent(lift_motor, 0.42, (int)((288 / 3) / (Math.PI*2)), 0);
    }


    private void arcadeDrive(Motor leftMotor, Motor rightMotor, double forwardSpeed, double turnSpeed) {
        leftMotor.set((forwardSpeed - turnSpeed) / 2D);
        rightMotor.set((forwardSpeed + turnSpeed) / 2D);
    }

    @Override
    public void every_tick() {
        GamepadEx gamepad = Inputs.gamepad1;

        double normalDriveXInput = gamepad.getLeftX();
        double normalDriveYInput = gamepad.getLeftY();
        double sidewaysDriveInput = gamepad.getRightX();

        //if joystick pos is less than this amount from in the middle, the robot doesn't move.
        final double DEAD_ZONE_SIZE = 0.2D;

        new GamepadButton(gamepad, PSButtons.SQUARE).whenActive(() -> {
            isSlowMode = !isSlowMode;
        });
        {
        }

        new GamepadButton(gamepad, PSButtons.CIRCLE).whenActive(() -> {
            isForwardOnlyMode = !isForwardOnlyMode;
        });


        if (isSlowMode) {
            for (double input: new double[]{normalDriveXInput, normalDriveYInput, sidewaysDriveInput}) {
                input /= 2D;
            }
        }

        if (isForwardOnlyMode) {
            normalDriveXInput = 0D;
        }

        //if the following variables are less than DEAD_ZONE_SIZE from 0, set them to be 0
        for (double input: new double[]{normalDriveXInput, normalDriveYInput, sidewaysDriveInput}) {
            input = -DEAD_ZONE_SIZE > input || input > DEAD_ZONE_SIZE ? input : 0D;
        }

        arcadeDrive(leftMotor, rightMotor, normalDriveXInput, normalDriveYInput);

        sidewaysMotor.set(sidewaysDriveInput);

        telemetry.addData("Normal Drive Train X", normalDriveXInput);
        telemetry.addData("Normal Drive Train Y", normalDriveYInput);
        telemetry.addData("Sideways Motor Value", sidewaysDriveInput);

        telemetry.addData("Slow Mode", isSlowMode);
        telemetry.addData("Forward Only Mode", isForwardOnlyMode);
        telemetry.addData("Dead Zone Size", DEAD_ZONE_SIZE);

        if(Math.abs(Inputs.gamepad1.getRightY()) != this.targetPosition) {
            this.targetPosition = Math.abs(Inputs.gamepad1.getRightY());
            lift.setHeight(this.targetPosition);
        } else {
            telemetry.addLine("[Lift] BUSY");
        }
        telemetry.addData("[Lift] Last set position to", this.targetPosition);
        telemetry.addData("[Lift] Position of gamepad", Math.abs(Inputs.gamepad1.getRightY()));

    }
}