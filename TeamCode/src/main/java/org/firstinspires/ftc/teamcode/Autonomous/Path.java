package org.firstinspires.ftc.teamcode.drive.Autonomous;

import org.firstinspires.ftc.teamcode.drive.modules.EditablePose2D;
import org.firstinspires.ftc.teamcode.drive.modules.Robot;

import java.lang.reflect.Array;
import java.util.ArrayList;

// maybe usable, idk  usable, needs some edits.

public class Path {
    public interface RobotAction {
        void Action();
    }

    public enum FollowMethods {
        TURN_THEN_FOLLOW, //Only use this one for now. The others are so bad.
        FOLLOW_THEN_TURN,
        FOLLOW_AND_TURN,
        TURN
    }

    private ArrayList<PathPoint> path = new ArrayList<PathPoint>();

    public Path(PathBuilder b){
        this.path = b.getPath();
    }

    public ArrayList<PathPoint> getPath(){
        return path;
    }

    /**
     * @param i index of the waypoint list
     * @return a pathpoint at index i
     */
    public PathPoint get(int i){
        return path.get(i);
    }

    public int getPathSize(){
        return path.size();
    }

    public void removePoint(int i){
        path.remove(i);
    }


    public static class PathPoint{

        private RobotAction a = null;
        private Waypoint w;
        private FollowMethods f = FollowMethods.TURN_THEN_FOLLOW;
        private double delayUntilNextPoint = 0;

        public void setRobotAction(RobotAction a){
            this.a = a;
        }

        public void setWaypoint(Waypoint w){
            this.w = w;
        }

        public Waypoint getWaypoint(){
            return w;
        }

        public void setFollowMethod(FollowMethods f){
            this.f = f;
        }

        public RobotAction getAction(){
            return this.a;
        }

        public double getDelayUntilNextPoint(){
            return delayUntilNextPoint;
        }

        public PathPoint(Waypoint w, FollowMethods f, RobotAction a, double delayUntilNextPoint){
            this.a = a;
            this.w = w;
            this.f = f;
            this.delayUntilNextPoint = delayUntilNextPoint;
        }

    }

    public static class PathBuilder{
        private ArrayList<PathPoint> path = new ArrayList<PathPoint>();

        public PathBuilder addNewPoint(Waypoint point){
            PathPoint p = new PathPoint(point, FollowMethods.TURN_THEN_FOLLOW, null, 0);
            path.add(p);
            return this;
        }

        public PathBuilder addNewActionButAddIfFreePathPoint(RobotAction a, double delay){
            if (!path.isEmpty()) {
                PathPoint p = path.get(path.size() - 1);
                if (p.getAction() == null){
                    p.setRobotAction(a);
                }
                else{
                    path.add(new PathPoint(null, FollowMethods.TURN_THEN_FOLLOW, a, delay));
                }
            }

            return this;
        }

        public PathBuilder addNewFullPoint(Waypoint w, FollowMethods f, RobotAction a, double delay){
            PathPoint p = new PathPoint(w, f, a, delay);
            path.add(p);
            return this;
        }

        public ArrayList<PathPoint> getPath(){
            return path;
        }

        public Path build(){
            return new Path(this);
        }

    }
}
