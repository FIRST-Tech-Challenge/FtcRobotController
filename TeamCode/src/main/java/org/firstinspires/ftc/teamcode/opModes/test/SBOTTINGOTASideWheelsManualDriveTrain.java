package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

/**
 * A program that controls a drive train has special wheels to go sideways.
 * Controls:
 *  Left Joystick: Control the drive train in a normal way.
 *  Right Joystick: The X value of this joystick makes the drive train go sideways.
 */

@Disabled
@TeleOp(name="Side Wheels Drive Train [sbottingota]", group="Demo")
public class SBOTTINGOTASideWheelsManualDriveTrain extends TeleOpModeBase {

    //private Telemetry telemetry;
    private DifferentialDrive normalDriveTrain;
    private DifferentialDrive sidewaysDriveTrain;

    @Override
    public void setup() {
        //telemetry = TelemetryContainer.getTelemetry();

        normalDriveTrain = new DifferentialDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1);
        sidewaysDriveTrain = new DifferentialDrive(HardwareMapContainer.motor2, HardwareMapContainer.motor3);
    }

    // Changes the -1 to 1 inputs that the gamepad returns into 0 to 1 inputs to be put into the arcadeDrive method
    private double changeRangeOfGamepadInput(double input) {
        return (input + 1) / 2;
    }

    @Override
    public void every_tick() {
        normalDriveTrain.arcadeDrive(changeRangeOfGamepadInput(Inputs.gamepad1.getLeftX()),
                changeRangeOfGamepadInput(Inputs.gamepad1.getLeftY()));

        double sidewaysDriveTrainInput = changeRangeOfGamepadInput(Inputs.gamepad1.getRightX());
        sidewaysDriveTrain.tankDrive(sidewaysDriveTrainInput, sidewaysDriveTrainInput);
    }
}
