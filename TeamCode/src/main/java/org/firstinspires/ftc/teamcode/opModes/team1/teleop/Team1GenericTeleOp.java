package org.firstinspires.ftc.teamcode.opModes.team1.teleop;

import com.arcrobotics.ftclib.drivebase.HDrive;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;

/**
 * This isn't an OpMode - create an instance of this in each team-specific OpMode
 */
public class Team1GenericTeleOp {
    TeamColour teamColour;
    HDrive drive;

    public void setup(TeamColour teamColor){
        this.teamColour = teamColor;
        // Get the third motor as a spinner motor
        drive = new HDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1, HardwareMapContainer.motor2);
    }

    public void every_tick(){
        double x_movement =  Inputs.gamepad1.getRightX();
        double y_movement =  Inputs.gamepad1.getRightY();
        // TODO: Get IMU heading and set turn
        drive.driveFieldCentric(x_movement, y_movement, 0, 0);
    }
}
