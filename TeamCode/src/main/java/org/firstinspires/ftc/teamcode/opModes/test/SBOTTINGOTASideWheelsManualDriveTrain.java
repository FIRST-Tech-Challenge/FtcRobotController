package org.firstinspires.ftc.teamcode.opModes.test;

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
@TeleOp(name="Side Wheels Drive Train [sbottingota]", group="Demo")
public class SBOTTINGOTASideWheelsManualDriveTrain extends TeleOpModeBase {

    private Telemetry telemetry;

    //private DifferentialDrive normalDriveTrain;

    private Motor leftMotor;
    private Motor rightMotor;

    private Motor sidewaysMotor;

    private boolean isSlowMode = false;
    private boolean isForwardOnlyMode = false;

    private final GamepadEx gamepad = Inputs.gamepad1;

    @Override
    public void setup() {
        telemetry = TelemetryContainer.getTelemetry();

        //normalDriveTrain = new DifferentialDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1);
        leftMotor = HardwareMapContainer.motor0;
        rightMotor = HardwareMapContainer.motor1;

        sidewaysMotor = HardwareMapContainer.motor2;

        new GamepadButton(gamepad, PSButtons.SQUARE).whenActive(() -> isSlowMode = !isSlowMode);

        new GamepadButton(gamepad, PSButtons.CIRCLE).whenActive(() -> isForwardOnlyMode = !isForwardOnlyMode);
    }

    // Changes the -1 to 1 inputs that the gamepad returns into 0 to 1 inputs to be put into the arcadeDrive method
//    private double changeRangeOfGamepadInput(double input) {
//
//        return (input + 1D) / 2D;
//    }

    private void arcadeDrive(Motor leftMotor, Motor rightMotor, double forwardSpeed, double turnSpeed) {
        leftMotor.set((forwardSpeed - turnSpeed) / 2D);
        rightMotor.set((forwardSpeed + turnSpeed) / 2D);
    }

    @Override
    public void every_tick() {

        final double[] inputs = {gamepad.getLeftY(), gamepad.getLeftX(), gamepad.getRightX()}; //forward speed, turn speed, sideways speed

        //if joystick pos is less than this amount from in the middle, the robot doesn't move.
        final double DEAD_ZONE_SIZE = 0.2D;

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

        sidewaysMotor.set(inputs[2]);

        telemetry.addData("Normal Drive Train Y", inputs[0]);
        telemetry.addData("Normal Drive Train X", inputs[1]);
        telemetry.addData("Sideways Motor Value", inputs[2]);

        telemetry.addData("Slow Mode", isSlowMode);
        telemetry.addData("Forward Only Mode", isForwardOnlyMode);
        telemetry.addData("Dead Zone Size", DEAD_ZONE_SIZE);
    }
}
