package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.PSButtons;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.opModes.team1.teleop.Team1GenericTeleOp;

@TeleOp(name="LAREINCExampleDrive2", group="LaReineC")
public class LAREINECExampleDriveTrain2 extends TeleOpModeBase {
    Team1GenericTeleOp teleop;

    DifferentialDrive drive;

    LAREINECJoystick leftJoystick;

    LAREINECJoystick rightJoystick;

    boolean isDrive;

    enum driveVersions {
        ARCADEDRIVE,
        TANKDRIVE,
    }

    @Override
    public void setup() {
        leftJoystick = new LAREINECJoystick(Inputs.gamepad1.getLeftX(), Inputs.gamepad1.getLeftY());
        rightJoystick =  new LAREINECJoystick(Inputs.gamepad1.getLeftX(), Inputs.gamepad1.getLeftY());
        drive = new DifferentialDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1);
        isDrive = false;
        // Runs once at INIT
    }
    public void driveMode(driveVersions mode) {
        switch (mode) {
            case TANKDRIVE:
                drive.tankDrive(leftJoystick.y, rightJoystick.y);
            case ARCADEDRIVE:
                drive.arcadeDrive(leftJoystick.y, leftJoystick.x);
        }
    }

    @Override
    public void every_tick() {
        teleop.every_tick();
        if (Inputs.gamepad1.wasJustPressed(PSButtons.TRIANGLE)) {
            isDrive = !isDrive;
            if (isDrive) {
                driveMode(driveVersions.ARCADEDRIVE);
                if (Inputs.gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    leftJoystick.y *= 2;
                    leftJoystick.x *= 2;
                }
                if (Inputs.gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    leftJoystick.y /= 2;
                    leftJoystick.x /= 2;
                }
            }
            if (!isDrive) {
                driveMode(driveVersions.TANKDRIVE);
                if (Inputs.gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    leftJoystick.y *= 2;
                    rightJoystick.y *= 2;
                }

                if (Inputs.gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    leftJoystick.y /= 2;
                    rightJoystick.y /= 2;
                }
            }
        }
    }
}

