package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.core.ROBOT_DATA.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.DriveMotor;
import org.firstinspires.ftc.teamcode.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryState;
import org.firstinspires.ftc.teamcode.trajectory.markers.Marker;
import org.firstinspires.ftc.teamcode.utils.MathFunctions;
import org.firstinspires.ftc.teamcode.utils.Pose2D;

import java.util.ArrayList;

public class RamseteFollower extends MecanumDrive{

    public RamseteFollower(HardwareMap hwMap, Pose2D pose, DriveMotor leftFrontDrive, DriveMotor rightFrontDrive, DriveMotor leftRearDrive, DriveMotor rightRearDrive) {
        super(hwMap, pose, leftFrontDrive, rightFrontDrive, leftRearDrive, rightRearDrive);
    }

    private void followPath(Pose2D target, double targetVelo, double targetRotVelo) {
        Pose2D error = new Pose2D(target.x - pose.x, target.y - pose.y, target.theta - Math.toRadians(pose.theta)).globalize(Math.toRadians(pose.theta));
        // Normalize the rotational error
        while(error.theta > Math.PI) { error.theta -= 2*Math.PI;}
        while(error.theta <= -Math.PI) {error.theta += 2*Math.PI;}

        error.theta *= RAMSETE_W;
        double k = 2 * RAMSETE_ZETA * Math.sqrt(targetRotVelo * targetRotVelo + RAMSETE_B*(targetVelo * targetVelo));
        setDifferentialVelo(targetVelo * Math.cos(error.theta) + k * error.x,
                              targetRotVelo + k*error.theta + RAMSETE_B*sinc(error.theta)*error.y);
    }

    public void executeTrajectory(Trajectory traj) {
        ElapsedTime timer = new ElapsedTime();
        ArrayList<Marker> markers = traj.getMarkerArray();

        double totalTime = traj.getLength();
        // Execute trajectory
        while (timer.seconds() <= totalTime) {
            TrajectoryState state = traj.get(timer.seconds());
            followPath(state.target, state.velocity, state.rotationalVelocity);
            for (Marker i : markers) {
                if (i.getTimeStamp() <= timer.seconds()) {
                    i.run();
                    markers.remove(i);
                }
            }
        }

    }

    private void setDifferentialVelo(double velo, double w) {
        runCommand(kinematics.toWheelSpeeds(new Pose2D(velo, 0, w)));
    }

    private double sinc(double x) {
        if (MathFunctions.epsEquals(x, 0)) {
            return 1.0 - 1.0 / 6.0 * x * x;
        }
        return Math.sin(x) / x;
    }
}
