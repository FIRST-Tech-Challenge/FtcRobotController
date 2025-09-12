package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import java.util.ArrayList;

// maybe usable, idk  usable, needs some edits.

public class Path {
    public interface RobotAction {
        void Action();
    }

    // This is the grave of the enum known as FollowMethod.
    //     It was useless because we were only using one
    //     type of "follow method" in the first place.

    private ArrayList<PathPoint> path = new ArrayList<PathPoint>();

    public Path(PathBuilder b){
        this.path = b.getPath();
    }

    // This is the grave of getPath, which I feel like should have
    //     been used but for some reason it wasn't.
    //     This version returned ArrayList<PathPoint>

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

    // This is the grave of removePoint() which I honestly feel like
    //     is useless unless the path was completely fucked up.


    public static class PathPoint{

        private RobotAction a = null;
        private Waypoint w;
        private double delayUntilNextPoint = 0;

        // The grave of setRobotAction(RobotAction a)
        //     Dunno what this was about -_- T-T ToT

        // The grave of setWayPoint(Waypoint w)
        //     The fact that we didn't use this feels kind of stupid
        //     It feels like we should be using this :|
        //     It was basically just "this.w = w" in case you wanna revive it

        public Waypoint getWaypoint(){
            return w;
        }

        public RobotAction getAction(){
            return this.a;
        }

        public double getDelayUntilNextPoint(){
            return delayUntilNextPoint;
        }

        public PathPoint(Waypoint w, RobotAction a, double delayUntilNextPoint){
            this.a = a;
            this.w = w;
            this.delayUntilNextPoint = delayUntilNextPoint;
        }

    }

    public static class PathBuilder{
        private ArrayList<PathPoint> path = new ArrayList<PathPoint>();

        // This is the grave of a few PathBuilders that nobody knows why they existed

        public PathBuilder addNewFullPoint(Waypoint w, RobotAction a, double delay){
            PathPoint p = new PathPoint(w, a, delay);
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
