package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;
import static java.lang.Math.atan2;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;

public class CatmulRomInterpolater {
    ArrayList<Pose2d> points;
    boolean isEndpoint = false;
    RFWaypoint ref;
    double offset = 0;

    public CatmulRomInterpolater(ArrayList<RFSegment> p_segments, int index) {
        points = new ArrayList();
        //generate first three points for interpolation, fill in if there are none
        if (index > 1) {
            points.add(p_segments.get(index - 2).getWaypoint().getTarget());
            points.add(p_segments.get(index - 1).getWaypoint().getTarget());
            points.add(p_segments.get(index).getWaypoint().getTarget());
        } else if (index == 1) {
            points.add(currentPose);
            points.add(p_segments.get(0).getWaypoint().getTarget());
            points.add(p_segments.get(1).getWaypoint().getTarget());
        } else {
            points.add(currentPose.minus(currentVelocity));
            points.add(currentPose);
            points.add(p_segments.get(0).getWaypoint().getTarget());
        }
        //generate last point for interpolation, fill in if there are none
        if (index + 1 < p_segments.size()) {
            points.add(p_segments.get(index + 1).getWaypoint().getTarget());
        } else {
            isEndpoint = true;
            points.add(points.get(2).minus(points.get(1)).plus(points.get(2)));
        }
        ref = p_segments.get(index).getWaypoint();
        offset = p_segments.get(index).getTangentOffset();
    }

    public RFWaypoint CRerpWaypoint() {
        if (ref.getDefinedness() < 3) {
            double targetVelocity = MAX_VEL;
            if (isEndpoint) {
                targetVelocity = 0;
            }
            if (ref.getDefinedness() < 2) {
                double endTangent = getEndTangent();
                if (ref.getDefinedness() == 0) {
                    return new RFWaypoint(new Pose2d(ref.getTarget().getX(), ref.getTarget().getY(), endTangent + offset), endTangent
                            , targetVelocity, ref.getDefinedness());
                }
                if (ref.getDefinedness() == 1) {
                    return new RFWaypoint(ref.getTarget(), ref.getTarget().getHeading() + offset, targetVelocity, ref.getDefinedness());
                }
            } else {
                return new RFWaypoint(ref.getTarget(), ref.getEndTangent(), targetVelocity, ref.getDefinedness());
            }
        } else {
            return ref;
        }
        return ref;
    }

    public double getEndTangent() {
        return atan2(terminalYVelocity(), terminalXVelocity());
    }

    public double terminalXVelocity() {
        return (0.5 * ((points.get(2).getX() - points.get(1).getX()) + 2 * (2 * points.get(1).getX() - 5 * points.get(1).getX()
                + 4 * points.get(2).getX() - points.get(3).getX()) + 3 * (-points.get(1).getX() + 3 * points.get(1).getX()
                - 3 * points.get(2).getX() + points.get(3).getX())));
    }

    public double terminalYVelocity() {
        return (0.5 * ((points.get(2).getY() - points.get(1).getY()) + 2 * (2 * points.get(1).getY() - 5 * points.get(1).getY()
                + 4 * points.get(2).getY() - points.get(3).getY()) * 1 + 3 * (-points.get(1).getY() + 3 * points.get(1).getY()
                - 3 * points.get(2).getY() + points.get(3).getY())));
    }
}
