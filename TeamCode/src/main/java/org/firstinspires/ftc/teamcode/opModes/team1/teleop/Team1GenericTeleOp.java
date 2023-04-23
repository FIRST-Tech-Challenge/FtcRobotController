package org.firstinspires.ftc.teamcode.opModes.team1.teleop;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.RevIMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.HDriveWrapper;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.Maths;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

/**
 * This isn't an OpMode - create an instance of this in each team-specific OpMode
 */
public class Team1GenericTeleOp {
    TeamColour teamColour;
    RevIMU imu;
    HDriveWrapper drive;

    public void setup(TeamColour teamColor){
        this.teamColour = teamColor;
        // Get the third motor as a spinner motor
        drive = new HDriveWrapper(new HDrive(
                HardwareMapContainer.motor0,
                HardwareMapContainer.motor1,
                HardwareMapContainer.motor2,
                0,
                Math.PI,
                Math.PI/2
        ), imu);
        imu = new RevIMU(HardwareMapContainer.getMap());
        imu.init();
    }

    public void every_tick(){
        // Note: directions of x and y on joystick different to directions on the field
        double strafe =  Inputs.gamepad1.getRightX();
        double forward =  Inputs.gamepad1.getRightY();
        // The direction of the left joystick is the desired direction of heading
        double x_joystick_turn = Inputs.gamepad1.getLeftX();
        double y_joystick_turn = Inputs.gamepad1.getLeftY();

        Telemetry t = TelemetryContainer.getTelemetry();
        t.addData("x", strafe);
        t.addData("y", forward);

        drive.updateTurn(x_joystick_turn, y_joystick_turn);
        drive.fieldOrientedDrive(strafe, forward);
    }
}
