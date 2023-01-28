package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;
import org.firstinspires.ftc.teamcode.opModes.team1.teleop.Team1GenericTeleOp;

@Disabled
@TeleOp(name="Drivetrain Demo [la-reine-c v2]", group="Demo")
public class LAREINECExampleDriveTrain2 extends TeleOpModeBase { // TODO: Research why the commands / wasJustPressed are not working
    DifferentialDrive drive;

    LAREINECJoystick leftJoystick;

    LAREINECJoystick rightJoystick;

    double speed;

    boolean isDrive;

    enum driveVersions {
        ARCADEDRIVE,
        TANKDRIVE,
    }

    @Override
    public void setup() {
        drive = new DifferentialDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1);
        isDrive = false;
        // Runs once at INIT

        speed = 1.0;

        // Add button events
        new GamepadButton(Inputs.gamepad1, PSButtons.TRIANGLE).whenPressed(new InstantCommand(() -> {
            TelemetryContainer.getTelemetry().addLine("Triangle pressed");
            this.isDrive = !this.isDrive;
        }));

        new GamepadButton(Inputs.gamepad1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(() -> {
                this.speed /= 2;
        }));
        new GamepadButton(Inputs.gamepad1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(() -> {
            this.speed *= 2;
        }));
    }
    public void driveMode(driveVersions mode) {
        // Get Joystick Positions
        leftJoystick = new LAREINECJoystick(Inputs.gamepad1.getLeftX(), Inputs.gamepad1.getLeftY());
        rightJoystick =  new LAREINECJoystick(Inputs.gamepad1.getLeftX(), Inputs.gamepad1.getLeftY());

        switch (mode) {
            case TANKDRIVE:
                drive.tankDrive(leftJoystick.y * this.speed, rightJoystick.y * this.speed);
            case ARCADEDRIVE:
                drive.arcadeDrive(leftJoystick.y * this.speed, leftJoystick.x);
        }
    }

    @Override
    public void every_tick() {

        TelemetryContainer.getTelemetry().addData("Speed Factor", this.speed);
        TelemetryContainer.getTelemetry().addData("Is Arcade Drive", isDrive);

        if (isDrive) {
            driveMode(driveVersions.ARCADEDRIVE);
        } else {
            driveMode(driveVersions.TANKDRIVE);
        }
    }
}

