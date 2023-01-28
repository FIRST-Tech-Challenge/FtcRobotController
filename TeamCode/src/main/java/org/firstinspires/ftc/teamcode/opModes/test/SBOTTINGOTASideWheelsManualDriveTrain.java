package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
 *  DPad Left and DPad Right: Make drive train go sideways.
 *
 *  Triangle: Toggle between normal speed and slow speed.
 */

@Disabled
@TeleOp(name="Side Wheels Drive Train [sbottingota]", group="Demo")
public class SBOTTINGOTASideWheelsManualDriveTrain extends TeleOpModeBase {

    //private Telemetry telemetry;
    private DifferentialDrive normalDriveTrain;
    private DifferentialDrive sidewaysDriveTrain;

    private boolean isSlowMode = false;

    @Override
    public void setup() {
        //telemetry = TelemetryContainer.getTelemetry();

        normalDriveTrain = new DifferentialDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1);
        sidewaysDriveTrain = new DifferentialDrive(HardwareMapContainer.motor2, HardwareMapContainer.motor2);
    }

    // Changes the -1 to 1 inputs that the gamepad returns into 0 to 1 inputs to be put into the arcadeDrive method
    private double changeRangeOfGamepadInput(double input) {
        return (input + 1D) / 2D;
    }

    @Override
    public void every_tick() {
        if (Inputs.gamepad1.wasJustPressed(PSButtons.TRIANGLE)) {
            isSlowMode = !isSlowMode;
        }

        double normalDriveX = Inputs.gamepad1.getLeftX();
        double normalDriveY = Inputs.gamepad1.getLeftY();

        double sidewaysDriveInput = 0D;

        if (Inputs.gamepad1.isDown(GamepadKeys.Button.DPAD_LEFT)) {
            sidewaysDriveInput -= 1D;
        }

        if (Inputs.gamepad1.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            sidewaysDriveInput += 1D;
        }

        if (isSlowMode) {
            normalDriveX /= 2D;
            normalDriveY /= 2D;

            sidewaysDriveInput /= 2D;
        }

        normalDriveX = changeRangeOfGamepadInput(normalDriveX);
        normalDriveY = changeRangeOfGamepadInput(normalDriveY);

        sidewaysDriveInput = changeRangeOfGamepadInput(sidewaysDriveInput);

        normalDriveTrain.arcadeDrive(normalDriveX, normalDriveY);

        sidewaysDriveTrain.tankDrive(sidewaysDriveInput, sidewaysDriveInput);
    }
}
