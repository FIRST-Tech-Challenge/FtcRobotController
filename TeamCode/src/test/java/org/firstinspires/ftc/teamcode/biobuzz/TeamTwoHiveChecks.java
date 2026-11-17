package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Injected tag/odometry fixtures, not real camera recognition or physical shot tests. */
public final class TeamTwoHiveChecks {
    private static final class IO implements RobotIO {
        double time,age,frame;boolean connected=true,valid=true,stop,freeze;int pipeline,count=4,pollen=4,reads;
        Pose pose;Target[] targets=new Target[0];Outputs out=new Outputs();
        IO(int alliance){pose=new FieldLayout().roleStart(alliance,true);}
        public double nowSeconds(){return time;}
        public boolean stopRequested(){return stop;}
        public Sensors readSensors(){reads++;if(!freeze)frame=time-age;return new Sensors(pose,time,valid,connected,count,pollen,new Vision(true,pipeline,frame,targets));}
        public void apply(Outputs o){out=o.copy();}
    }
    private static Target[] tags(int alliance,boolean far,double height){
        int id=(far?new FieldLayout().farTags:new FieldLayout().nearTags)[alliance][0];
        return new Target[]{new Target(id,0,0,height),new Target(id+1,0,0,height)};
    }
    private static void check(boolean ok,String why){if(!ok)throw new AssertionError(why);}
    private static void step(IO io,TeamTwoHiveAuto auto){io.time+=.04;auto.tick();}
    private static void noShot(IO io,String why){check(io.out.shooterPower==0&&io.out.feederPower==0,why);}
    private static void allOff(IO io,String why){noShot(io,why);check(io.out.forward==0&&io.out.strafeRight==0&&io.out.turnClockwise==0&&io.out.intakePower==0,why);}
    private static TeamTwoHiveAuto atWall(IO io,int alliance){
        TeamTwoHiveAuto auto=new TeamTwoHiveAuto(io,new FieldLayout());auto.start(alliance,.55);
        Pose shot=auto.destination();check(shot==new FieldLayout().teamTwoWallHive[alliance]||shot.distance(new FieldLayout().teamTwoWallHive[alliance])==0,"First destination is opposite wall");
        check(shot.x==new FieldLayout().hive[alliance].x&&shot.y==-new FieldLayout().hive[alliance].y,"Mirror Team One across the field");
        double frontY=shot.y+9.05*Math.sin(shot.heading);check(Math.abs(Math.abs(frontY)-72)<.1,"Front touches nominal opposite wall, rear points at Hive");
        io.pose=shot;step(io,auto);check(auto.getState()==TeamTwoHiveAuto.State.VERIFY_HIVE,"Verify after arriving; do not start blind feed");return auto;
    }
    private static void permit(IO io,TeamTwoHiveAuto auto,int alliance){
        io.targets=tags(alliance,true,52.02);io.age=0;io.pipeline=0;io.connected=true;io.freeze=false;
        for(int i=0;i<8&&auto.getState()==TeamTwoHiveAuto.State.VERIFY_HIVE;i++)step(io,auto);
        check(auto.getState()==TeamTwoHiveAuto.State.SHOOTING,"Fresh stable own far Cell UP permits spin-up");
        for(int i=0;i<35&&io.out.feederPower==0;i++)step(io,auto);
        check(io.out.feederPower==1&&io.out.shooterPower==.55,"Only then run shooter/feed");
    }
    public static void main(String[] args){
        for(int alliance=0;alliance<2;alliance++){
            IO io=new IO(alliance);TeamTwoHiveAuto auto=atWall(io,alliance);
            io.targets=tags(alliance,false,52.02);for(int i=0;i<8;i++)step(io,auto);noShot(io,"Wrong Hive tilt blocks shots");
            io.targets=tags(alliance,false,36.57);for(int i=0;i<8;i++)step(io,auto);noShot(io,"Inferred far-up without visible far tags is insufficient");
            int far=new FieldLayout().farTags[alliance][0],near=new FieldLayout().nearTags[alliance][0];
            io.targets=new Target[]{new Target(near,0,0,36.57),new Target(near+1,0,0,36.57),new Target(far,0,0,52.02)};
            for(int i=0;i<8;i++)step(io,auto);noShot(io,"Near-down confirmation plus one far tag cannot replace two far-tag evidence");
            io.targets=new Target[]{new Target(far,0,0,52.02),new Target(far,0,0,52.02)};
            for(int i=0;i<8;i++)step(io,auto);noShot(io,"Duplicate detections of the same ID do not count as two far tags");
            io.targets=tags(1-alliance,true,52.02);io.time=11;for(int i=0;i<8;i++)step(io,auto);noShot(io,"Opponent tags never permit shots, even after ten seconds");
            io.targets=tags(alliance,true,Double.NaN);for(int i=0;i<8;i++)step(io,auto);noShot(io,"Uncalibrated heights block shooting");
            io.targets=tags(alliance,true,52.02);io.pipeline=1;for(int i=0;i<8;i++)step(io,auto);noShot(io,"Wrong camera pipeline blocks shooting");
            io.pipeline=0;io.age=.6;for(int i=0;i<8;i++)step(io,auto);noShot(io,"Stale tags block shooting");
            io.age=0;io.freeze=true;io.frame=io.time;for(int i=0;i<4;i++)step(io,auto);noShot(io,"Repeated frame cannot confirm raised Cell");
            permit(io,auto,alliance);
            Pose verified=io.pose;io.pose=new Pose(verified.x+2,verified.y,verified.heading);step(io,auto);noShot(io,"Fresh tags do not authorize feeding away from the shooting pose");
            io.pose=verified;permit(io,auto,alliance);
            io.connected=false;step(io,auto);noShot(io,"Camera loss immediately shuts off feed and shooter");
            io.time=26;auto.tick();check(auto.getState()==TeamTwoHiveAuto.State.PARK,"Parking overrides missing tags/camera by 26 seconds");check(io.out.intakePower==0,"Parking cancels intake");
            io.time=30;int reads=io.reads;auto.tick();check(!auto.isRunning()&&io.reads==reads,"Hard stop before another SDK/sensor read");allOff(io,"Hard stop clears every output");

            io=new IO(alliance);auto=atWall(io,alliance);permit(io,auto,alliance);
            io.age=.6;step(io,auto);noShot(io,"Stale frame while feeding pauses both mechanisms");
            io.time+=2;step(io,auto);noShot(io,"Elapsed time cannot bypass paused gate");
            io.age=0;permit(io,auto,alliance);
            io.count=io.pollen=3;step(io,auto);check(io.out.feederPower==1,"Authorized four-pollen batch continues as its measured count drops");
            io.count=io.pollen=0;step(io,auto);
            check(auto.getState()==TeamTwoHiveAuto.State.TO_FLOWER,"Measured empty completes the admitted first load, then goes to Flower");
            io.pose=auto.destination();step(io,auto);check(auto.getState()==TeamTwoHiveAuto.State.PRESS_FLOWER,"Reach Flower trigger");
            io.time+=.41;step(io,auto);check(auto.getState()==TeamTwoHiveAuto.State.CLEAR_FLOWER,"Contact pulse is bounded");
            io.pose=auto.destination();step(io,auto);check(auto.getState()==TeamTwoHiveAuto.State.TO_OUTLET_LANE,"Clear Flower outlet");
            io.pose=auto.destination();step(io,auto);check(auto.getState()==TeamTwoHiveAuto.State.TO_PICKUP,"Enter open outlet lane without crossing pollen");
            io.pose=auto.destination();step(io,auto);check(auto.getState()==TeamTwoHiveAuto.State.INTAKE,"Reach Flower intake view");
            io.pipeline=1;io.targets=new Target[]{new Target(-1,0,-10,Double.NaN,2,"yellow")};
            io.count=io.pollen=3;for(int i=0;i<35;i++)step(io,auto);
            check(auto.getState()==TeamTwoHiveAuto.State.INTAKE,"A timer cannot turn three pollen into four or leave the Flower");
            io.targets=new Target[0];step(io,auto);check(io.out.intakePower==0&&io.out.forward==0,"No fresh pollen image cancels intake/translation");
            io.count=io.pollen=4;step(io,auto);
            check(auto.getState()==TeamTwoHiveAuto.State.TO_HIVE,"Four measured pollen return to the wall, never a timed assumption");
            check(auto.destination().distance(new FieldLayout().teamTwoWallHive[alliance])==0,"Both shot batches use identical wall pose");
            io.pose=auto.destination();io.pipeline=0;step(io,auto);check(auto.getState()==TeamTwoHiveAuto.State.VERIFY_HIVE,"Require a new gate after Flower, not old permission");
            io.targets=tags(alliance,false,52.02);for(int i=0;i<8;i++)step(io,auto);noShot(io,"Wrong orientation blocks the Flower load too");
            permit(io,auto,alliance);
            io.targets=tags(alliance,false,52.02);step(io,auto);noShot(io,"A tipping/wrong Cell immediately stops final feed");
            for(int i=0;i<8;i++)step(io,auto);
            check(auto.getState()==TeamTwoHiveAuto.State.SETTLING,"Independently confirmed second tilt ends final feeding");

            io=new IO(alliance);auto=atWall(io,alliance);io.stop=true;reads=io.reads;auto.tick();check(!auto.isRunning()&&io.reads==reads,"STOP before SDK reads");allOff(io,"STOP cancels outputs");
            io=new IO(alliance);auto=atWall(io,alliance);io.valid=false;step(io,auto);check(!auto.isRunning(),"Invalid odometry never permits blind navigation");allOff(io,"Invalid odometry stops outputs");

            for(int[] counts:new int[][]{{-1,-1},{0,0},{1,1},{2,2},{3,3},{4,3},{4,-1}}){
                io=new IO(alliance);io.count=counts[0];io.pollen=counts[1];auto=atWall(io,alliance);
                io.targets=tags(alliance,true,52.02);for(int i=0;i<40;i++)step(io,auto);
                check(auto.getState()==TeamTwoHiveAuto.State.VERIFY_HIVE,"Only a full four-pollen load may start shooting");noShot(io,"Partial, mixed or unknown inventory blocks shooter/feed");
                io.time=26;auto.tick();check(auto.getState()==TeamTwoHiveAuto.State.PARK,"Count gate cannot prevent force parking");
            }

            // Drive-command integration with synthetic fresh tags, not physical firing evidence.
            io=new IO(alliance);auto=new TeamTwoHiveAuto(io,new FieldLayout());auto.start(alliance,.55);
            int feedStarts=0,intakeTicks=0;boolean previousFeed=false,sawFlower=false;double parkAt=-1,lastModelShot=-1;
            for(int i=0;i<1501&&auto.isRunning();i++){
                // Test fixture supplies explicit inventory events; production never counts by time.
                io.pipeline=io.out.pipeline;
                if(io.out.feederPower>0&&io.time-lastModelShot>=.38&&io.count>0){io.count--;io.pollen--;lastModelShot=io.time;}
                if(auto.getState()==TeamTwoHiveAuto.State.INTAKE&&++intakeTicks>=20)io.count=io.pollen=4;
                io.targets=io.pipeline==1?new Target[]{new Target(-1,0,-10,Double.NaN,2,"yellow")}:tags(alliance,true,52.02);auto.tick();
                if(io.out.feederPower>0){
                    check(io.pose.distance(new FieldLayout().teamTwoWallHive[alliance])<.75,"Feed only at the opposite-wall pose");
                    if(!previousFeed)feedStarts++;
                }
                previousFeed=io.out.feederPower>0;sawFlower|=auto.getState()==TeamTwoHiveAuto.State.INTAKE;
                if(auto.getState()==TeamTwoHiveAuto.State.PARK&&parkAt<0)parkAt=io.time;
                Pose beforePose=io.pose;Outputs o=io.out;double h=Pose.angle(io.pose.heading-o.turnClockwise*1.65*.02);
                io.pose=new Pose(io.pose.x+(o.forward*Math.cos(h)+o.strafeRight*Math.sin(h))*25*.02,
                        io.pose.y+(o.forward*Math.sin(h)-o.strafeRight*Math.cos(h))*25*.02,h);
                // Model the physical Flower post stopping the deliberate bounded press.
                if(auto.getState()==TeamTwoHiveAuto.State.PRESS_FLOWER&&!new FieldLayout().allowed(io.pose))io.pose=beforePose;
                check(new FieldLayout().allowed(io.pose),"Integrated route collision: alliance="+alliance+" t="+io.time+" state="+auto.getState()+" pose="+io.pose.x+","+io.pose.y+","+io.pose.heading+" output="+o.forward+","+o.strafeRight+","+o.turnClockwise);io.time+=.02;
            }
            check(feedStarts==2&&sawFlower,"Complete both camera-gated feed cycles and Flower pickup");
            check(parkAt>0&&parkAt<=26.02&&io.time<30&&io.pose.distance(new FieldLayout().teamTwoGardenSidePark[alliance])<=2,"Complete parking before 30 seconds");allOff(io,"Arrival stays stopped");
            System.out.println("Team Two alliance "+alliance+": two verified feed cycles; park starts "+parkAt+", finishes "+io.time);
        }
        System.out.println("Team Two opposite-wall geometry, both-batch own-tag gates, stale/lost/wrong/uncalibrated evidence, paused feed, STOP and 26/30-second parking checks passed.");
    }
}
