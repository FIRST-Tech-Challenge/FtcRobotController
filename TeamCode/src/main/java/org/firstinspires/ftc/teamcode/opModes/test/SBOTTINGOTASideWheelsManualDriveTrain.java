package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
    private DifferentialDrive normalDriveTrain;
    private Motor sidewaysMotor;

    private boolean isSlowMode = false;
    private boolean isForwardOnlyMode = false;

    @Override
    public void setup() {
        telemetry = TelemetryContainer.getTelemetry();

        normalDriveTrain = new DifferentialDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1);
        sidewaysMotor = HardwareMapContainer.motor2;
    }

    // Changes the -1 to 1 inputs that the gamepad returns into 0 to 1 inputs to be put into the arcadeDrive method
    private double changeRangeOfGamepadInput(double input) {
        return (input + 1D) / 2D;
    }

    @Override
    public void every_tick() {
        double normalDriveXInput = Inputs.gamepad1.getLeftX();
        double normalDriveYInput = Inputs.gamepad1.getLeftY();
        double sidewaysDriveInput = Inputs.gamepad1.getRightX();

        //if joystick pos is less than this amount from in the middle, the robot doesn't move.
        final double DEAD_ZONE_SIZE = 0.1D;

        if (Inputs.gamepad1.wasJustPressed(PSButtons.SQUARE)) {
            isSlowMode = !isSlowMode;
        }

        if (Inputs.gamepad1.wasJustPressed(PSButtons.CIRCLE)) {
            isForwardOnlyMode = !isForwardOnlyMode;
        }

        if (isSlowMode) {
            for (double input: new double[]{normalDriveXInput, normalDriveYInput, sidewaysDriveInput}) {
                input /= 2D;
            }
        }

        if (isForwardOnlyMode) {
            for (double input: new double[]{normalDriveXInput, normalDriveYInput, sidewaysDriveInput}) {
                input = Math.min(input, 0D);
            }
        }

        //if the following variables are less than DEAD_ZONE_SIZE from 0, set them to be 0
        for (double input: new double[]{normalDriveXInput, normalDriveYInput, sidewaysDriveInput}) {
            input = -DEAD_ZONE_SIZE > input || input > DEAD_ZONE_SIZE ? input : 0D;
        }

        //sidewaysInput isn't included as motors are meant to have -1 to 1 inputs, not 0-1 inputs
        for (double input: new double[]{normalDriveXInput, normalDriveYInput}) {
            input = changeRangeOfGamepadInput(input);
        }

        normalDriveTrain.arcadeDrive(normalDriveXInput, normalDriveYInput);

        sidewaysMotor.set(sidewaysDriveInput);

        telemetry.addData("Normal Drive Train X (0 to 1)", normalDriveXInput);
        telemetry.addData("Normal Drive Train Y (0 to 1)", normalDriveYInput);
        telemetry.addData("Sideways Motor Value (-1 to 1)", sidewaysDriveInput);

        telemetry.addData("Slow Mode", isSlowMode);
        telemetry.addData("Forward Only Mode", isForwardOnlyMode);
        telemetry.addData("Dead Zone Size", DEAD_ZONE_SIZE);
    }
}
