package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.maths.PIDcontroller;

public class Turret {
    //do we want positional PID turret or Velocity PID Turret???
    //Positional determine desired position for us to be inline with target
    //Velocity: adjust velocity to stay in line with turret
    //Problems
    //Positional: harder to keep track of robot position and goal position
    //Velocity: easier for oscillations to occur

    //Velocity: uses a limelight mounted to the turret to determine where it should move.
    //  Whether it needs to move more left or right is determined by offset of
    //  limelight from center of apriltag on the goal
    //  this ends in us setting up a velocity PID with the velocity factor
    //  changing based on position

    //Positional: uses the robots pose on the field and the known pose of the goal to determine
    //  the angle that the turret should be at. this gives us a target pos (angle) for our turret
    //  to be at. with a positional PID to do this. This also lets us mount vision elsewhere.
    //  making it easier hardware wise since the April tag is almost directly above the
    //  robot by 3 feet.

    //We are using a positional Controller YAY

    //I guess we should use a PIDF controller
    private double targetAngleDegrees; //degrees
    private double currentAngleDegrees;
    private PIDFCoefficients pidCoefficients;
    private PIDFCoefficients pidController;

    //motors be important
    private DcMotorEx azimuthMotor;

    ///set target position
    /// @param targetAngle target angle in degrees
    public void setTargetAngleDegrees(double targetAngle) {
        targetAngleDegrees = targetAngle;
    }

    public double getTargetAngleDegrees(){
        return targetAngleDegrees;
    }


}
