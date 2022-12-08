package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.command.SubsystemBase;

/**
 * Controls and updates the robot's odometry from the wheel encoders and IMU.
 * Because the system uses dead reckoning (no external encoders), the error of the
 * system will increase over time, particularly if there is a contact with another robot.
 */
public class OdometrySystem extends SubsystemBase {
}
