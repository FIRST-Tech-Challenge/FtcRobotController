package org.darbots.darbotsftclib.libcore.purepursuit.followers;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitWayPoint;

public class PurePursuitPathFollower {
    public static class FollowInformation{
        public PurePursuitWayPoint purePursuitWayPoint;
        public RobotPoint2D pursuitPoint;
        public double normalizedFollowSpeed;
        public boolean pidEnabled;

        public FollowInformation(PurePursuitWayPoint wayPoint, RobotPoint2D followPoint, double normalizedFollowSpeed, boolean PIDEnabled){
            this.pursuitPoint = followPoint;
            this.purePursuitWayPoint = wayPoint;
            this.normalizedFollowSpeed = Math.abs(normalizedFollowSpeed);
            this.pidEnabled = PIDEnabled;
        }
        public FollowInformation(FollowInformation oldInfo){
            this.pursuitPoint = oldInfo.pursuitPoint;
            this.purePursuitWayPoint = oldInfo.purePursuitWayPoint;
            this.normalizedFollowSpeed = oldInfo.normalizedFollowSpeed;
            this.pidEnabled = oldInfo.pidEnabled;
        }
    }


}
