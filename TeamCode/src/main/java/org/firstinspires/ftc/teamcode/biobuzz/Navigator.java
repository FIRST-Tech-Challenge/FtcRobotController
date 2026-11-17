package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Pose;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Outputs;

/** Collision-aware navigation with resumable planning; no slow-device timeout. */
public final class Navigator {
    private static final int[][] NEIGHBORS={{4,0},{-4,0},{0,4},{0,-4},{4,4},{4,-4},{-4,4},{-4,-4}};
    private final FieldLayout field;
    private final BooleanSupplier cancelled;
    private final LongSupplier clock;
    private int stage;
    private List<Pose> route;
    private Pose turnPose,wallRetreat;
    private boolean wallDepartureChecked;
    private TurnSearch turning;
    private PathSearch path;
    private boolean planning;
    private boolean scripted;
    private int alliance;
    public Navigator(FieldLayout field){this(field,()->Thread.currentThread().isInterrupted());}
    public Navigator(FieldLayout field,BooleanSupplier cancelled){this(field,cancelled,System::nanoTime);}
    public Navigator(FieldLayout field,BooleanSupplier cancelled,LongSupplier clock){this.field=field;this.cancelled=cancelled;this.clock=clock;}
    private void checkStop(){if(cancelled.getAsBoolean()||Thread.currentThread().isInterrupted())throw new java.util.concurrent.CancellationException("Navigation cancelled by STOP");}
    private final class Slice {
        final long end=clock.getAsLong()+20000000L;
        int units;
        boolean more(){checkStop();return ++units<=4096&&clock.getAsLong()<end;}
    }
    public void reset(){stage=0;route=null;wallRetreat=null;wallDepartureChecked=false;turnPose=null;turning=null;path=null;planning=false;}
    public boolean isPlanning(){return planning;}
    public void useScriptedRoute(boolean enabled,int alliance){this.scripted=enabled;this.alliance=alliance;reset();}
    public List<Pose> route(){return route==null?new ArrayList<Pose>():new ArrayList<Pose>(route);}
    /** Pending searches return zero drive, not an exception or false arrival. */
    public Outputs move(Pose p,Pose target){
        checkStop();
        Outputs retreat=retreatFromFrontWall(p,target);
        if(retreat!=null)return retreat;
        if(scripted)return moveScripted(p,target);
        checkStop();planning=false;Slice slice=new Slice();
        if(stage==0){
            if(turning==null)turning=new TurnSearch(p,target.heading);
            while(!turning.done&&slice.more())turning.step();
            if(!turning.done){planning=true;return new Outputs();}
            if(turning.result==null)throw new IllegalStateException("No reachable turn clearance");
            turnPose=turning.result;route=turning.approach;turning=null;stage=p.distance(turnPose)<.01?2:1;
        }
        if(stage==1){if(p.distance(turnPose)<.6){stage=2;route=null;path=null;}else return follow(p,turnPose,turnPose.heading,slice);}
        if(stage==2){
            checkStop();if(!field.canRotate(p,target.heading))throw new IllegalStateException("Turn blocked");
            double a=Pose.angle(target.heading-p.heading);
            if(Math.abs(a)>Math.toRadians(1.5)){Outputs o=new Outputs();o.turnClockwise=-FieldLayout.clip(Math.toDegrees(a)*.08,field.turnPower);return o;}
            stage=3;route=null;path=null;
        }
        if(p.distance(target)<.65&&Math.abs(Pose.angle(target.heading-p.heading))<Math.toRadians(2))return null;
        return follow(p,target,target.heading,slice);
    }
    /** Leave a wall-facing shot straight backwards before strafing or turning. */
    private Outputs retreatFromFrontWall(Pose p,Pose target){
        if(!wallDepartureChecked){
            wallDepartureChecked=true;
            if(p.distance(target)<.65&&Math.abs(Pose.angle(target.heading-p.heading))<Math.toRadians(2))return null;
            boolean frontAtWall=Math.abs(p.x)>58&&Math.cos(p.heading)*Math.signum(p.x)>.9
                    ||Math.abs(p.y)>58&&Math.sin(p.heading)*Math.signum(p.y)>.9;
            if(frontAtWall){
                wallRetreat=new Pose(p.x-6*Math.cos(p.heading),p.y-6*Math.sin(p.heading),p.heading);
                if(!field.clear(p,wallRetreat,p.heading))throw new IllegalStateException("No safe backward wall-departure corridor");
            }
        }
        if(wallRetreat==null)return null;
        if(p.distance(wallRetreat)<.5){wallRetreat=null;return null;}
        Outputs out=new Outputs();
        out.forward=-Math.min(field.drivePower,Math.min(.65,Math.max(.15,p.distance(wallRetreat)*.45)));
        double error=Pose.angle(wallRetreat.heading-p.heading);
        if(field.canRotate(p,wallRetreat.heading))out.turnClockwise=-FieldLayout.clip(Math.toDegrees(error)*.08,.2);
        return out;
    }
    private Pose fixedPoint(double x,double y,double heading){return new Pose(alliance==0?x:-x,alliance==0?y:-y,heading);}
    private Outputs moveScripted(Pose p,Pose target){
        checkStop();planning=false;
        if(stage==0){
            if(field.canRotate(p,target.heading))turnPose=p;
            else{
                double best=Double.POSITIVE_INFINITY;
                double[][] clearance={{-12.75,-56},{-32,-56},{-40,-52},{-52,52},{-52,-52},{-12.75,56},{-32,56},{-36,52},{-40,52},{-56,-56},{-52,-32},{-56,-40},{-52,-36},{-52,-40},{-48,-36},{-52,-24},{-52,-28},{-48,-24}};
                for(double[] xy:clearance){
                    checkStop();Pose q=fixedPoint(xy[0],xy[1],p.heading);double distance=p.distance(q);
                    if(distance<best&&field.canRotate(q,target.heading)&&field.clear(p,q,p.heading)){best=distance;turnPose=q;}
                }
                if(turnPose==null)throw new IllegalStateException("Fixed turn approach blocked: verify start/odometry/field calibration");
            }
            stage=p.distance(turnPose)<.01?2:1;
        }
        if(stage==1){
            if(p.distance(turnPose)<.6){stage=2;route=null;}
            else{if(route==null)route=fixedPath(p,turnPose,p.heading);return driveRoute(p,turnPose,p.heading);}
        }
        if(stage==2){
            checkStop();if(!field.canRotate(p,target.heading))throw new IllegalStateException("Fixed-route turn blocked");
            double a=Pose.angle(target.heading-p.heading);
            if(Math.abs(a)>Math.toRadians(1.5)){Outputs o=new Outputs();o.turnClockwise=-FieldLayout.clip(Math.toDegrees(a)*.08,field.turnPower);return o;}
            stage=3;route=null;
        }
        if(p.distance(target)<.65&&Math.abs(Pose.angle(target.heading-p.heading))<Math.toRadians(2))return null;
        if(route==null)route=fixedPath(p,target,target.heading);
        return driveRoute(p,target,target.heading);
    }
    /** A handful of validated, alliance-mirrored corridor templates; no graph search. */
    private List<Pose> fixedPath(Pose start,Pose goal,double heading){
        Pose rear=fixedPoint(-12.75,56,heading),upper=fixedPoint(-40,36,heading),lower=fixedPoint(-46,-44,heading),bypass=fixedPoint(-52,-8,heading);
        Pose highCorner=fixedPoint(-52,52,heading),lowCorner=fixedPoint(-52,-52,heading),highCenter=fixedPoint(-12.75,52,heading),lowCenter=fixedPoint(-12.75,-52,heading);
        Pose[][] templates={{highCenter,goal},{lowCenter,goal},{highCorner,highCenter,goal},{lowCorner,lowCenter,goal},{highCorner,lowCorner,lowCenter,goal},{lowCorner,highCorner,highCenter,goal},{goal},{bypass,goal},{rear,upper,goal},{lower,upper,goal},{upper,lower,goal},{upper,goal},{lower,goal},{rear,upper,lower,goal}};
        List<Pose> bestPath=null;double bestLength=Double.POSITIVE_INFINITY;
        for(Pose[] template:templates){
            checkStop();Pose previous=start;boolean clear=true;double length=0;
            for(Pose waypoint:template){checkStop();if(!field.clear(previous,waypoint,heading)){clear=false;break;}length+=previous.distance(waypoint);previous=waypoint;}
            if(clear&&length<bestLength){bestLength=length;bestPath=new ArrayList<Pose>(Arrays.asList(template));}
        }
        if(bestPath!=null)return bestPath;
        throw new IllegalStateException("Fixed corridor blocked: verify start/odometry/field calibration");
    }
    private Outputs driveRoute(Pose p,Pose target,double heading){
        while(route.size()>1&&(p.distance(route.get(0))<(scripted?.05:.6)||!scripted&&field.clear(p,route.get(1),heading))){checkStop();route.remove(0);}
        Pose q=route.get(0);double dx=q.x-p.x,dy=q.y-p.y,c=Math.cos(p.heading),s=Math.sin(p.heading);
        double gain=scripted?.45:.35;
        Outputs o=new Outputs();o.forward=(dx*c+dy*s)*gain;o.strafeRight=(dx*s-dy*c)*gain;
        double mag=Math.hypot(o.forward,o.strafeRight);if(mag>field.drivePower){o.forward*=field.drivePower/mag;o.strafeRight*=field.drivePower/mag;}
        o.turnClockwise=-FieldLayout.clip(Math.toDegrees(Pose.angle(heading-p.heading))*.08,field.turnPower);return o;
    }
    private Outputs follow(Pose p,Pose target,double heading,Slice slice){
        if(route==null){
            if(path==null)path=new PathSearch(p,target,heading);
            while(!path.done&&slice.more())path.step();
            if(!path.done){planning=true;return new Outputs();}
            route=path.result;path=null;if(route==null)throw new IllegalStateException("No collision-free route");
        }
        return driveRoute(p,target,heading);
    }
    private final class TurnSearch {
        final Pose start;final double heading;
        final PriorityQueue<Pose> candidates;
        int cell;boolean initialized,done;Pose result,candidate;PathSearch search;List<Pose> approach;
        TurnSearch(Pose start,double heading){this.start=start;this.heading=heading;candidates=new PriorityQueue<Pose>(Comparator.comparingDouble((Pose p)->p.distance(start)).thenComparingDouble(p->p.x).thenComparingDouble(p->p.y));}
        void step(){
            checkStop();
            if(!initialized){initialized=true;if(field.canRotate(start,heading)){result=start;done=true;return;}}
            // Test rotation only for the nearest candidates actually considered,
            // not all 841 grid points (each rotation samples 25 footprints).
            if(cell<29*29){int i=cell++;Pose p=new Pose(-56+(i/29)*4,-56+(i%29)*4,start.heading);if(field.allowed(p))candidates.add(p);return;}
            if(search==null){candidate=candidates.poll();if(candidate==null){done=true;return;}if(!field.canRotate(candidate,heading))return;search=new PathSearch(start,candidate,start.heading);}
            search.step();
            if(search.done){if(search.result!=null){result=candidate;approach=search.result;done=true;}search=null;}
        }
    }
    private static final class Node {
        final Pose p;final int x,y,index;double cost=Double.POSITIVE_INFINITY;Node parent;
        Node(int x,int y,int index,double h){this.x=x;this.y=y;this.index=index;p=new Pose(x,y,h);}
        String key(){return x+":"+y;}
    }
    private final class PathSearch {
        final Pose start,goal;final double heading;
        final Map<String,Node> map=new HashMap<String,Node>();
        final PriorityQueue<Node> nearStart,nearGoal;
        PriorityQueue<Node> open;
        final Set<String> closed=new HashSet<String>();
        int phase,cell,nearTries;Node first,last,trace;boolean done;List<Pose> result;
        PathSearch(Pose start,Pose goal,double heading){
            this.start=start;this.goal=goal;this.heading=heading;
            nearStart=new PriorityQueue<Node>(Comparator.comparingDouble((Node n)->n.p.distance(start)).thenComparingInt(n->n.index));
            nearGoal=new PriorityQueue<Node>(Comparator.comparingDouble((Node n)->n.p.distance(goal)).thenComparingInt(n->n.index));
        }
        void step(){
            checkStop();if(done)return;
            if(phase==0){
                if(!start.finite()||!goal.finite()||!Double.isFinite(heading)||Math.abs(start.x)>72.001||Math.abs(start.y)>72.001||!field.allowed(new Pose(goal.x,goal.y,heading))){done=true;return;}
                if(field.clear(start,goal,heading)){result=new ArrayList<Pose>(Arrays.asList(goal));done=true;return;}phase=1;return;
            }
            if(phase==1){
                if(cell<33*33){int i=cell++;Node n=new Node(-64+(i/33)*4,-64+(i%33)*4,i,heading);if(field.allowed(n.p)){map.put(n.key(),n);nearStart.add(n);nearGoal.add(n);}return;}
                phase=2;nearTries=0;return;
            }
            if(phase==2||phase==3){
                PriorityQueue<Node> near=phase==2?nearStart:nearGoal;Node n=near.poll();
                if(n==null||nearTries++>=24){done=true;return;}
                if(field.clear(phase==2?start:goal,n.p,heading)){
                    if(phase==2){first=n;phase=3;nearTries=0;}
                    else{last=n;open=new PriorityQueue<Node>(Comparator.comparingDouble(q->q.cost+q.p.distance(last.p)));first.cost=0;open.add(first);phase=4;}
                }return;
            }
            if(phase==5){if(trace==null){done=true;return;}result.add(0,trace.p);trace=trace.parent;return;}
            Node n=open.poll();if(n==null){done=true;return;}if(closed.contains(n.key()))return;
            if(n==last){result=new ArrayList<Pose>();result.add(goal);trace=n;phase=5;return;}
            closed.add(n.key());
            for(int[] offset:NEIGHBORS){
                int dx=offset[0],dy=offset[1];checkStop();Node q=map.get((n.x+dx)+":"+(n.y+dy));
                if(q==null||closed.contains(q.key())||!field.clear(n.p,q.p,heading))continue;
                double cost=n.cost+Math.hypot(dx,dy);if(cost<q.cost){open.remove(q);q.cost=cost;q.parent=n;open.add(q);}
            }
        }
    }
    /** Synchronous diagnostic/test helper. Runtime movement uses resumable searches above. */
    public List<Pose> plan(Pose start,Pose goal,double heading){PathSearch search=new PathSearch(start,goal,heading);while(!search.done){checkStop();search.step();}return search.result;}
}
