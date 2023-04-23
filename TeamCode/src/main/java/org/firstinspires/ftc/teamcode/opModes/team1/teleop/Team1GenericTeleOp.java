package org.firstinspires.ftc.teamcode.opModes.team1.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;

import org.firstinspires.ftc.teamcode.components.HDriveWrapper;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

/**
 * This isn't an OpMode - create an instance of this in each team-specific OpMode
 */
public class Team1GenericTeleOp {
    TeamColour teamColour;
    RevIMU imu;
    HDriveWrapper drive;

    // If pushed down, the robot can freely turn
    // Otherwise, snap to the nearest cardinal direction
    GamepadButton turnModeSwitch;

    public void setup(TeamColour teamColor){
        this.teamColour = teamColor;
        imu = new RevIMU(HardwareMapContainer.getMap());
        imu.init();
        // Get the third motor as a spinner motor
        drive = new HDriveWrapper(new HDrive(
                HardwareMapContainer.motor0,
                HardwareMapContainer.motor1,
                HardwareMapContainer.motor2,
                0,
                Math.PI,
                Math.PI/2
        ), imu);

        // Reset IMU heading on button press
        // This can also be activated by the button on the back of the joystick
        // That makes the robot consider the direction it was facing the front (pointing to the right from driver's point of view)
        Inputs.gamepad1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new InstantCommand(() -> {
            imu.reset();
        }));

        turnModeSwitch = Inputs.gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON);
    }

    public void every_tick(){
        // Drivetrain
        // Note: directions of x and y on joystick different to directions on the field
        double strafe =  Inputs.gamepad1.getRightX();
        double forward =  -Inputs.gamepad1.getRightY();
        // The direction of the left joystick is the desired direction of heading
        double x_joystick_turn = Inputs.gamepad1.getLeftX();
        double y_joystick_turn = Inputs.gamepad1.getLeftY();

        // If not pressed, snap to the nearest cardinal direction
        drive.setTurnSnap(!turnModeSwitch.get());

        MultipleTelemetry t = TelemetryContainer.getTelemetry();
        t.addData("x", strafe);
        t.addData("y", forward);
        t.addData("Version", 1);

        drive.updateTurn(x_joystick_turn, y_joystick_turn);
        drive.fieldOrientedDrive(strafe, forward);
    }
}
