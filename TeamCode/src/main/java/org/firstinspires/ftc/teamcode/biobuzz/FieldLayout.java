package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Pose;

/** Shared geometry and waypoints. Defaults match this simulator, not measured robot calibration. */
public final class FieldLayout {
    public final Pose[] hive={fromBrowser(-12.75,62.9,Math.PI),fromBrowser(12.75,-62.9,0)};
    // Oblique return shot keeps the Garden/Flower loop on the alliance wall side.
    // Heading points at the raised Cell's AprilTag center; same .395 / 66.5-degree arc.
    public final Pose[] farHive={fromBrowser(-53,40,Math.PI-.9843162068450441),fromBrowser(53,-40,2*Math.PI-.9843162068450441)};
    public final Pose[] garden={fromBrowser(-58,60.8,Math.PI),fromBrowser(58,-60.8,0)};
    // Side Flowers are closer to the Garden/shot route than the opposite rear Flower.
    // Corner approach triggers release without putting the chassis across the outlet.
    public final Pose[] flower={fromBrowser(-58,35.5,0),fromBrowser(58,-35.5,Math.PI)};
    public final Pose[] nearFlower={fromBrowser(-35.5,-58,-Math.PI/2),fromBrowser(35.5,58,Math.PI/2)};
    // Nearest Flower to Team Two's corrected wall shot is the end-wall Flower.
    // First retreat from the Hive wall, then line up with the Flower before
    // making the short forward trigger contact. This avoids a long diagonal
    // across the field and preserves the wall-shot heading throughout.
    public final Pose[] teamTwoHiveRetreat={new Pose(-12.75,50,Math.PI/2),new Pose(12.75,-50,-Math.PI/2)};
    public final Pose[] teamTwoFlowerAlign={new Pose(-35.5,50,Math.PI/2),new Pose(35.5,-50,-Math.PI/2)};
    // Trigger with a corner, clear the post, then face its inward pollen outlet.
    public final Pose[] teamTwoFlowerTouch={new Pose(-35.5,57.5,Math.PI/2),new Pose(35.5,-57.5,-Math.PI/2)};
    public final Pose[] teamTwoFlowerClear={new Pose(-42,55,Math.PI/2),new Pose(42,-55,-Math.PI/2)};
    public final Pose[] teamTwoFlowerLane={new Pose(-42,30,Math.PI/2),new Pose(42,-30,-Math.PI/2)};
    public final Pose[] teamTwoFlowerPickup={new Pose(-24,30,Math.PI/2),new Pose(24,-30,-Math.PI/2)};
    // Inset enough to fit the chassis at any heading while overlapping the Loading Zone.
    public final Pose[] loadingPark={new Pose(-59,30,0),new Pose(59,-30,Math.PI)};
    // Team One takes the half nearest its starting wall, not globally smaller Y.
    // Team Two takes the remaining half; modeled chassis stay separated at all headings.
    public final Pose[] teamOneStartSidePark={new Pose(-59,21,0),new Pose(59,-21,Math.PI)};
    public final Pose[] teamTwoGardenSidePark={new Pose(-59,51,0),new Pose(59,-51,Math.PI)};
    // Opposite Team One's starting wall, same Hive X. Front faces the wall;
    // rear-facing shooter points into the Hive. 62.9 matches Team One's wall inset.
    public final Pose[] teamTwoWallHive={new Pose(-12.75,62.9,Math.PI/2),new Pose(12.75,-62.9,-Math.PI/2)};
    /** Same placements as autonomous-modes.js; Java coordinates are X, -browser Z. */
    public Pose roleStart(int alliance,boolean teamTwo) {
        if(alliance<0||alliance>1)throw new IllegalArgumentException("Invalid alliance");
        return teamTwo?teamTwoWallHive[alliance]:hive[alliance];
    }
    public Pose parkingTarget(Pose current,int alliance) {
        Pose location=loadingPark[alliance];
        return new Pose(location.x,location.y,current.heading);
    }
    public double drivePower=1.0,turnPower=1.0;
    // BIOBUZZ TU03 p76: Red rear 30..33 / audience 34..37;
    // Blue audience 38..41 / rear 42..45. Near is the corrected initial raised Cell (audience for Red, rear for Blue).
    public final int[][] nearTags={{34,35,36,37},{42,43,44,45}}, farTags={{30,31,32,33},{38,39,40,41}};
    // Nominal tag-plane heights at the two stable Hive positions. Measure on hardware.
    public double upperTagHeight=52.02,lowerTagHeight=36.57,tagHeightTolerance=1.5;
    public double tagHeightMotionTolerance=.20; // inches of drift allowed during confirmation
    public final List<double[]> obstacles=new ArrayList<double[]>();
    public FieldLayout() {
        for(double x:new double[]{-24.73,24.73})for(double y:new double[]{-15.5,15.5})obstacles.add(new double[]{x,y,1,4.9});
        obstacles.addAll(Arrays.asList(new double[]{-69,-24,2.25,2.25},new double[]{69,24,2.25,2.25},new double[]{-24,69,2.25,2.25},new double[]{24,-69,2.25,2.25}));
    }
    public static Pose fromBrowser(double x,double z,double theta) { return new Pose(x,-z,Pose.angle(theta+Math.PI/2)); }
    public boolean allowed(Pose p) {
        if(!p.finite()||Math.abs(p.x)>72.001||Math.abs(p.y)>72.001)return false;
        double t=p.heading-Math.PI/2,c=Math.cos(t),s=Math.sin(t);
        for(double x:new double[]{-8.75,8.75})for(double z:new double[]{-9.05,8.055})
            if(Math.abs(p.x+c*x+s*z)>72.001||Math.abs(-p.y-s*x+c*z)>72.001)return false;
        for(double[] b:obstacles){
            double dx=b[0]-p.x,dz=-b[1]+p.y,a=8.8,d=9.15;
            if(Math.abs(dx)<a*Math.abs(c)+d*Math.abs(s)+b[2]&&Math.abs(dz)<a*Math.abs(s)+d*Math.abs(c)+b[3]
                &&Math.abs(dx*c-dz*s)<a+b[2]*Math.abs(c)+b[3]*Math.abs(s)
                &&Math.abs(dx*s+dz*c)<d+b[2]*Math.abs(s)+b[3]*Math.abs(c))return false;
        }
        return true;
    }
    public boolean clear(Pose a,Pose b,double heading) {
        if(!Double.isFinite(heading)||!allowed(new Pose(a.x,a.y,heading))||!allowed(new Pose(b.x,b.y,heading)))return false;
        int n=Math.max(1,(int)Math.ceil(a.distance(b)));
        for(int i=0;i<=n;i++){if(Thread.currentThread().isInterrupted())throw new java.util.concurrent.CancellationException("Collision check cancelled");double t=(double)i/n;if(!allowed(new Pose(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,heading)))return false;}
        return true;
    }
    public boolean canRotate(Pose p,double heading) {
        double a=Pose.angle(heading-p.heading);
        for(int i=0;i<=24;i++)if(!allowed(new Pose(p.x,p.y,p.heading+a*i/24)))return false;
        return true;
    }
    public Pose park(Pose p) {
        double t=p.heading-Math.PI/2,c=Math.abs(Math.cos(t)),s=Math.abs(Math.sin(t));
        double x=72-8.75*c-9.05*s,y=72-8.75*s-9.05*c;
        Pose best=p;double min=Double.POSITIVE_INFINITY;
        Pose[] choices={new Pose(-x,clip(p.y,y),p.heading),new Pose(x,clip(p.y,y),p.heading),new Pose(clip(p.x,x),-y,p.heading),new Pose(clip(p.x,x),y,p.heading)};
        for(Pose q:choices)if(allowed(q)&&p.distance(q)<min){best=q;min=p.distance(q);}return best;
    }
    public static double clip(double value,double max) { return Math.max(-max,Math.min(max,value)); }
}
