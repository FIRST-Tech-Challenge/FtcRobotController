package org.firstinspires.ftc.teamcode.opModes.testing;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

@Disabled
@TeleOp(name="DriveTrainTest",group="Test")
public class TestOpMode extends TeleOpModeBase {
    public DifferentialDrive driveTrain;
    public Telemetry telemetry = TelemetryContainer.getTelemetry();


    private enum DriveModes {
        ARCADE_DRIVE,
        TANK_DRIVE
    }

    private DriveModes driveMode = DriveModes.ARCADE_DRIVE;

    private int speedMultiplier = 1;

    @Override
    public void setup() {
        driveTrain = new DifferentialDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1);
    }

    @Override
    public void every_tick() {
        if (Inputs.gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            driveMode = DriveModes.ARCADE_DRIVE;
        } else if (Inputs.gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            driveMode = DriveModes.TANK_DRIVE;
        }

        if (Inputs.gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            speedMultiplier /= 2;
        } else if (Inputs.gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            speedMultiplier *= 2;
        }


        if (driveMode == DriveModes.ARCADE_DRIVE) {
            driveTrain.arcadeDrive(Inputs.gamepad1.getLeftY() * speedMultiplier,
                    Inputs.gamepad1.getLeftX() * speedMultiplier);

            telemetry.addData("Let's print the joystick x", Inputs.gamepad1.getLeftX());
            telemetry.addData("Let's print the joystick y", Inputs.gamepad1.getLeftY());
        } else {
            driveTrain.tankDrive(Inputs.gamepad1.getLeftY() * speedMultiplier,
                    Inputs.gamepad1.getRightY() * speedMultiplier);

            telemetry.addData("Let's print the left joystick y", Inputs.gamepad1.getLeftY());
            telemetry.addData("Let's print the right joystick y", Inputs.gamepad1.getRightY());
        }

        telemetry.addData("Let's print the speed multiplier", speedMultiplier);
    }
}