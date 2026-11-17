package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Hardware-controller decisions with injected observations; not a physical camera/robot test. */
public final class TeamOnePollenChecks {
    private static final class IO implements RobotIO {
        double time,age;boolean connected=true,valid=true,stop,freeze;int pipeline,count=-1,reads;
        Pose pose;Target[] targets;Outputs out=new Outputs();double frame;
        IO(int alliance){pose=new FieldLayout().hive[alliance];targets=tags(alliance,true);}
        public double nowSeconds(){return time;}
        public boolean stopRequested(){return stop;}
        public Sensors readSensors(){reads++;if(!freeze)frame=time-age;return new Sensors(pose,time,valid,connected,count,new Vision(true,pipeline,frame,targets));}
        public void apply(Outputs o){out=o.copy();}
    }
    private static Target[] tags(int alliance,boolean up){int id=new FieldLayout().nearTags[alliance][0];double h=up?52.02:36.57;return new Target[]{new Target(id,0,0,h),new Target(id+1,0,0,h)};}
    private static Target ball(double tx,double area,String color){return new Target(-1,tx,0,Double.NaN,area,color);}
    private static void check(boolean ok,String why){if(!ok)throw new AssertionError(why);}
    private static void step(IO io,TeamOnePollenAuto auto){io.time+=.04;io.pipeline=io.out.pipeline;auto.tick();}
    private static void stopped(IO io,String why){check(io.out.forward==0&&io.out.strafeRight==0&&io.out.turnClockwise==0&&io.out.shooterPower==0&&io.out.feederPower==0&&io.out.intakePower==0,why);}
    private static TeamOnePollenAuto waiting(IO io,int alliance){
        TeamOnePollenAuto auto=new TeamOnePollenAuto(io,new FieldLayout());auto.surveyBeforePickup=false;auto.recoverPollenAfterTilt=true;auto.start(alliance,.55);
        boolean fed=false;
        for(int i=0;i<130&&auto.getState()==TeamOnePollenAuto.State.SHOOTING;i++){step(io,auto);fed|=io.out.feederPower>0;}
        check(fed,"Unknown/zero inventory cannot skip preload feed");
        check(auto.getState()==TeamOnePollenAuto.State.OBSERVE_TILT,"Move off fence for tilt view after timed preload attempt");
        io.pose=auto.destination();step(io,auto);check(auto.getState()==TeamOnePollenAuto.State.WAIT_TILT,"Stationary observation before pickup");
        io.time+=1;step(io,auto);check(auto.getState()==TeamOnePollenAuto.State.WAIT_TILT,"Time alone does not confirm a tilt");
        return auto;
    }
    private static TeamOnePollenAuto searching(IO io,int alliance){
        TeamOnePollenAuto auto=waiting(io,alliance);
        io.targets=tags(1-alliance,false);for(int i=0;i<6;i++)step(io,auto);
        check(auto.getState()==TeamOnePollenAuto.State.WAIT_TILT,"Opponent tag heights cannot confirm own Hive tilt");
        int own=new FieldLayout().nearTags[alliance][0];io.targets=new Target[]{new Target(own,0,0),new Target(own+1,0,0)};
        for(int i=0;i<6;i++)step(io,auto);
        check(auto.getState()==TeamOnePollenAuto.State.WAIT_TILT,"Uncalibrated/missing tag heights cannot enable collection");
        io.targets=tags(alliance,false);io.age=.6;for(int i=0;i<6;i++)step(io,auto);
        check(auto.getState()==TeamOnePollenAuto.State.WAIT_TILT,"Stale own tag heights cannot confirm a tilt");
        io.age=0;io.freeze=true;io.frame=io.time;for(int i=0;i<4;i++)step(io,auto);
        check(auto.getState()==TeamOnePollenAuto.State.WAIT_TILT,"Repeated frame cannot confirm a tilt");io.freeze=false;
        for(int i=0;i<8&&auto.getState()==TeamOnePollenAuto.State.WAIT_TILT;i++)step(io,auto);
        check(auto.getState()==TeamOnePollenAuto.State.CLEAR_PICKUP,"Three fresh stable own-Hive frames confirm tilt");
        io.pose=auto.destination();step(io,auto);check(auto.getState()==TeamOnePollenAuto.State.SEARCH,"Begin camera search after view clearance");
        check(io.out.pipeline==1,"Zero-power state transition retains color pipeline");return auto;
    }
    public static void main(String[] args){
        for(int alliance=0;alliance<2;alliance++){
            IO io=new IO(alliance);TeamOnePollenAuto auto=searching(io,alliance);
            io.targets=new Target[]{ball(-5,.8,"yellow"),ball(5,2,"yellow"),ball(-20,40,"blue")};
            step(io,auto);step(io,auto);check(io.out.forward==auto.approachPower&&io.out.strafeRight>0&&io.out.intakePower==1,"Reuse largest-pollen approach; do not chase larger nectar");
            io.age=.6;step(io,auto);stopped(io,"Stale ball frame cancels movement and intake");
            io.age=0;io.connected=false;step(io,auto);stopped(io,"Disconnected camera cancels recovery");
            io.time=26;auto.tick();check(auto.getState()==TeamOnePollenAuto.State.PARK,"26-second park overrides lost camera");
            check(io.out.intakePower==0&&io.out.feederPower==0&&io.out.shooterPower==0,"Parking clears every mechanism");
            Pose park=auto.destination();check(park.x==(alliance==0?-59:59)&&park.y==(alliance==0?21:-21),"Team One nearest-start parking target");
            check(new FieldLayout().loadingPark[alliance].y==(alliance==0?30:-30),"Legacy generic parking remains unchanged");
            check(new FieldLayout().allowed(park),"Lower-half parking pose fits field");
            for(int h=0;h<24;h++){
                double heading=h*Math.PI/12,t=heading-Math.PI/2;
                double reach=8.75*Math.abs(Math.sin(t))+9.05*Math.abs(Math.cos(t));
                check(alliance==0?park.y+reach+auto.parkingToleranceInches<36:park.y-reach-auto.parkingToleranceInches> -36,"Nearest-start half includes two-inch arrival tolerance");
                check(new FieldLayout().allowed(new Pose(park.x,park.y,heading)),"Parking footprint fits at every arrival heading");
            }
            io.pose=park;step(io,auto);check(!auto.isRunning(),"Parking arrival latches stop");stopped(io,"Parked robot cannot spin or creep");
            step(io,auto);stopped(io,"Parking stop remains latched");

            io=new IO(alliance);io.count=0;auto=waiting(io,alliance);io.connected=false;io.time=26;auto.tick();check(auto.getState()==TeamOnePollenAuto.State.PARK,"No tilt / no camera still parks at 26 seconds");
            io.time=30;int reads=io.reads;auto.tick();check(!auto.isRunning()&&io.reads==reads,"30-second stop before additional sensor reads");stopped(io,"30-second hard cutoff");

            io=new IO(alliance);auto=searching(io,alliance);io.targets=new Target[]{ball(0,13,"yellow")};step(io,auto);step(io,auto);
            check(io.out.intakePower==1&&io.out.forward==0,"Close pollen uses bounded intake with no feed");
            for(int i=0;i<25;i++)step(io,auto);
            check(auto.getState()==TeamOnePollenAuto.State.REPOSITION&&io.count==-1,"Close-target dwell ends without inventing pickup count");

            io=new IO(alliance);auto=searching(io,alliance);io.count=4;step(io,auto);check(auto.getState()==TeamOnePollenAuto.State.WAITING,"Measured full inventory stops collection");stopped(io,"Full inventory cancels outputs");
            io.time=26;auto.tick();check(auto.getState()==TeamOnePollenAuto.State.PARK,"Full inventory cannot prevent scheduled park");

            io=new IO(alliance);auto=searching(io,alliance);io.stop=true;reads=io.reads;auto.tick();check(!auto.isRunning()&&io.reads==reads,"STOP checked before camera/odometry reads");stopped(io,"Manual STOP");
            io=new IO(alliance);auto=searching(io,alliance);io.valid=false;step(io,auto);check(!auto.isRunning(),"Invalid odometry stops rather than blind parking");stopped(io,"Odometry safety");

            // Integrate drive commands, not teleported navigation goals. Tags are test inputs.
            io=new IO(alliance);auto=new TeamOnePollenAuto(io,new FieldLayout());auto.start(alliance,.55);
            double parkAt=-1;
            for(int i=0;i<1501&&auto.isRunning();i++){
                io.pipeline=io.out.pipeline;io.targets=io.pipeline==0?tags(alliance,auto.getState()!=TeamOnePollenAuto.State.WAIT_TILT):new Target[0];
                auto.tick();
                if(auto.getState()==TeamOnePollenAuto.State.PARK&&parkAt<0)parkAt=io.time;
                Outputs o=io.out;double h=Pose.angle(io.pose.heading-o.turnClockwise*1.65*.02);
                io.pose=new Pose(io.pose.x+(o.forward*Math.cos(h)+o.strafeRight*Math.sin(h))*25*.02,
                        io.pose.y+(o.forward*Math.sin(h)-o.strafeRight*Math.cos(h))*25*.02,h);
                check(new FieldLayout().allowed(io.pose),"Integrated route remains collision-clear");io.time+=.02;
            }
            check(parkAt>0&&parkAt<=26.02,"Integrated run starts parking by 26 seconds");
            check(io.time<30&&io.pose.distance(new FieldLayout().teamOneStartSidePark[alliance])<=2,"Integrated run reaches start-side-half park before 30 seconds");
            stopped(io,"Integrated arrival latches every actuator OFF");
            System.out.println("Team One integrated navigation alliance "+alliance+": park at "+parkAt+", finish "+io.time);
        }
        System.out.println("Team One red/blue: timed preloads, fresh own-Hive tilt, largest pollen, unknown counts, camera loss, bounded intake, start-side-half parking and 26/30-second/STOP guards passed.");
    }
}
