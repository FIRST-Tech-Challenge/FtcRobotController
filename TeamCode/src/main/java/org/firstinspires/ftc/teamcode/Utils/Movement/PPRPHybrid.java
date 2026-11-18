package org.firstinspires.ftc.teamcode.Utils.Movement;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class PPRPHybrid {

    public static Pose lastPose;
    static float timeDiff = 0.3f;
    static Follower follow;
    public static Telemetry tem = null;

    public static Path path(Position[] path) {
        ArrayList<Path> paths = new ArrayList<>();
        ArrayList<Pose> poses = new ArrayList<>();
        float timePerPoint = (path[1].getTimeStamp()-path[0].getTimeStamp())/1000.0f;
        int i = 0;
        while(i < path.length-1) {
            int a = 0;
            while(a < timeDiff/timePerPoint && path[i+a].getType() == 0 && i+a < path.length-1)a++;
            Pose s = poseFromPosi(path[i].round(3));
            Pose e = poseFromPosi(path[i+a].round(3));
            Position c0 = new Position(path[i+1].x()-path[i].x(),path[i+1].y()-path[i].y(),0);
            c0 = c0.sized((timeDiff)*3.0f); c0 = c0.offset(path[i]);
            Pose c0p = poseFromPosiNoR(c0.round(3));
            Position c1 = new Position(path[i+a-1].x()-path[i+a].x(),path[i+a-1].y()-path[i+a].y(),0);
            c1 = c1.sized((timeDiff)*3.0f); c1 = c1.offset(path[i+a]);
            Pose c1p = poseFromPosiNoR(c1.round(3));
            Path p = curve(s,c0p,c1p,e).linear(s,e);
            paths.add(p);
            poses.add(s); poses.add(c0p); poses.add(c1p); poses.add(e);
            i+=a;
            if(tem != null) {
                tem.addData("I",i);
                tem.addData("s",s);
                tem.addData("c0",c0p);
                tem.addData("c1",c1p);
                tem.addData("e",e);
                tem.update();
                long start = System.currentTimeMillis();
                while(System.currentTimeMillis()-start < 100) {}
            }
            if(i > path.length-5 && i < path.length-1) i = path.length-5;
        }
        if(tem != null) {
            for (int a = 0; a < poses.size(); a+=4) {
                tem.addData("path "+(a/4),
                        "s "+poses.get(a).toString()+
                                ", c0 "+poses.get(a+1)+
                                ", c1 "+poses.get(a+2)+
                                ", e "+poses.get(a+3)
                );
            }
            tem.update();
        }
        lastPose = poses.get(poses.size()-1);
        return Paths.path(paths.toArray(new Path[0]));
    }

    public static void runPath(Position[] path) {
        follow.follow(path(path));
    }

    public static Pose poseFromPosi(Position p) {
        return new Pose(p.x(),p.y(),Math.toRadians(-p.r()+90));
    }
    public static Pose poseFromPosiNoR(Position p) {
        return new Pose(p.x(),p.y());
    }

    public static void setFollower(Follower follower) {follow = follower;}
    public static Follower getFollower() {return follow;}

}
