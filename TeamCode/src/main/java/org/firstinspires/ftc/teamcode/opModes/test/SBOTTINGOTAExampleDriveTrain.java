package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

/**
 * A simple program to control a drive train.
 *
 * Controls:
 * Left joystick for arcade drive ({@link DifferentialDrive#arcadeDrive(double, double)})
 * Both joysticks for tank drive ({@link DifferentialDrive#tankDrive(double, double)})
 *
 * X to toggle between arcade and tank drive.
 * Y to toggle between inputting a 0 to 1 value or -1 to 1 value into the current drive mode (test).
 *
 * R1 and L1 to make the robot go faster and slower respectively.
 */
@Disabled
@TeleOp(name="Drivetrain Demo [sbottingota]", group="Demo")
public class SBOTTINGOTAExampleDriveTrain extends TeleOpModeBase {
    public DifferentialDrive driveTrain;
    public Telemetry telemetry;


    private enum DriveModes {
        ARCADE_DRIVE,
        TANK_DRIVE
    }

    private enum InputTypes {
        MINUS_ONE_TO_ONE,
        ZERO_TO_ONE
    }

    private DriveModes driveMode = DriveModes.ARCADE_DRIVE;
    private InputTypes inputType = InputTypes.ZERO_TO_ONE;

    private int speedMultiplier = 1;

    @Override
    public void setup() {
        telemetry = TelemetryContainer.getTelemetry();
        driveTrain = new DifferentialDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1);
    }

    @Override
    public void every_tick() {
        if (Inputs.gamepad1.wasJustPressed(GamepadKeys.Button.X)) {
            if (driveMode.equals(DriveModes.TANK_DRIVE)) {
                driveMode = DriveModes.ARCADE_DRIVE;
            } else {
                driveMode = DriveModes.TANK_DRIVE;
            }
        }

        if (Inputs.gamepad1.wasJustPressed(GamepadKeys.Button.Y)) {
            if (inputType.equals(InputTypes.MINUS_ONE_TO_ONE)) {
                inputType = InputTypes.ZERO_TO_ONE;
            } else {
                inputType = InputTypes.MINUS_ONE_TO_ONE;
            }
        }

        if (Inputs.gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            speedMultiplier /= 2;
        } else if (Inputs.gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            speedMultiplier *= 2;
        }


        if (driveMode.equals(DriveModes.ARCADE_DRIVE)) {
            double joystickX = Inputs.gamepad1.getLeftX();
            double joystickY = Inputs.gamepad1.getLeftY();


            driveTrain.arcadeDrive(joystickY * speedMultiplier, joystickX * speedMultiplier);

            telemetry.addData("Joystick x", joystickX);
            telemetry.addData("Joystick y", joystickY);
        } else {
            double leftJoystickY = Inputs.gamepad1.getLeftY();
            double rightJoystickY = Inputs.gamepad1.getRightY();

            if (inputType.equals(InputTypes.ZERO_TO_ONE)) {
                leftJoystickY = (leftJoystickY + 1) / 2;
                rightJoystickY = (rightJoystickY + 1) / 2;
            }

            driveTrain.tankDrive(leftJoystickY * speedMultiplier,
                    rightJoystickY * speedMultiplier);

            telemetry.addData("Let's print the left joystick y", leftJoystickY);
            telemetry.addData("Let's print the right joystick y", rightJoystickY);
        }

        telemetry.addData("Let's print the speed multiplier", speedMultiplier);
    }
}