package org.firstinspires.ftc.teamcode.opModes.team1.teleop;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.RevIMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    HDrive drive;

    public void setup(TeamColour teamColor){
        this.teamColour = teamColor;
        // Get the third motor as a spinner motor
        drive = new HDrive(HardwareMapContainer.motor0, HardwareMapContainer.motor1, HardwareMapContainer.motor2, 0, Math.PI, Math.PI/2);
        imu = new RevIMU(HardwareMapContainer.getMap());
        imu.init();
    }

    public void every_tick(){
        // Note: x and y on joystick different to field
        double x_movement =  Inputs.gamepad1.getRightY();
        double y_movement =  Inputs.gamepad1.getRightX();
        // The direction of the left joystick is the desired direction of heading
        double x_joystick_turn = Inputs.gamepad1.getLeftX();
        double y_joystick_turn = Inputs.gamepad1.getLeftY();
        Vector2d turn_joystick_vector = new Vector2d(y_joystick_turn, -x_joystick_turn);

        double desired_heading = turn_joystick_vector.angle();
        // Convert to degrees
        desired_heading *= 180 / Math.PI;

        double heading = imu.getAbsoluteHeading();
        double turn_strength = turn_joystick_vector.magnitude();

        Telemetry t = TelemetryContainer.getTelemetry();
        t.addData("x", x_movement);
        t.addData("y", y_movement);

        t.addData("heading", heading);
        t.addData("desired heading", desired_heading);

        double heading_error = heading - desired_heading;

        t.addData("heading error", heading_error);

        double turn;

        // Do not turn if in rest position
        if (turn_strength < 0.2) turn = 0;
        else turn = -0.02 * Maths.clamp(heading_error, -10, 10);

        t.addData("turn", turn);

        // TODO: Get IMU heading and set turn
        drive.driveFieldCentric(x_movement, y_movement, turn, heading);
    }
}
