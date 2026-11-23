package org.firstinspires.ftc.teamcode.biobuzz;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Injected observations verify decisions, not physical intake travel or odometry calibration. */
public final class GardenAndWaitChecks {
    static final class IO implements RobotIO {
        double time,frameAge;int count=0,pipeline;Pose pose;Target[] targets={};Outputs out=new Outputs();boolean stop;
        boolean connected=true,frameAvailable=true;
        public double nowSeconds(){return time;}
        public boolean stopRequested(){return stop;}
        public Sensors readSensors(){return new Sensors(pose,time,true,connected,count,count,new Vision(targets.length>0,pipeline,time-frameAge,targets,0,-1,frameAvailable));}
        public void apply(Outputs o){out=o.copy();pipeline=o.pipeline;}
    }
    static void check(boolean ok,String why){if(!ok)throw new AssertionError(why);}
    static void wheelsOff(IO io){check(io.out.forward==0&&io.out.strafeRight==0&&io.out.turnClockwise==0,"Stationary wait must not creep or spin");}
    static void tick(IO io,RobotController auto){io.time+=.04;auto.tick();}
    static Target[] tags(FieldLayout f,int a,boolean far){int[] ids=far?f.farTags[a]:f.nearTags[a];return new Target[]{new Target(ids[0],0,0,52.02),new Target(ids[1],0,0,52.02)};}
    static TeamOnePollenAuto garden(IO io,FieldLayout f,int a,double seconds){
        io.pose=f.hive[a];io.targets=tags(f,a,false);
        TeamOnePollenAuto auto=new TeamOnePollenAuto(io,f);auto.spinUpSeconds=0;auto.feedSeconds=.08;auto.gardenAdvanceSeconds=seconds;auto.start(a,.5);
        for(int i=0;i<80&&auto.getState()!=TeamOnePollenAuto.State.TO_GARDEN;i++)tick(io,auto);
        check(auto.getState()==TeamOnePollenAuto.State.TO_GARDEN,"Finish verified preload attempt");
        io.pose=auto.destination();tick(io,auto);check(auto.getState()==TeamOnePollenAuto.State.INTAKE_GARDEN,"Reach Garden approach");io.targets=new Target[0];return auto;
    }
    public static void main(String[] args){
        if(args.length==1&&args[0].equals("yellow-window")){yellowWindowChecks();return;}
        for(int a=0;a<2;a++){
            FieldLayout f=new FieldLayout();IO io=new IO();TeamOnePollenAuto one=garden(io,f,a,5);
            tick(io,one);check(io.out.forward==one.gardenAdvancePower&&io.out.turnClockwise==0&&io.out.intakePower==1,"Advance into real intake even after pollen falls below camera");
            io.targets=new Target[]{new Target(-1,1.5,0,Double.NaN,5,"yellow")};tick(io,one);
            check(io.out.turnClockwise==0&&io.out.strafeRight==0,"Ignore small image jitter");
            Pose entry=io.pose;io.pose=new Pose(entry.x+18*Math.cos(entry.heading),entry.y+18*Math.sin(entry.heading),entry.heading);
            tick(io,one);check(one.getState()==TeamOnePollenAuto.State.WAITING,"Measured stroke ends collection travel");wheelsOff(io);
            io.time=25.9;one.tick();check(one.getState()==TeamOnePollenAuto.State.WAITING,"Completed routine holds until 26 seconds");wheelsOff(io);
            io.time=26;one.tick();check(one.getState()==TeamOnePollenAuto.State.PARK&&io.out.intakePower==0,"26-second park shuts intake off");
            io=new IO();one=garden(io,f,a,.16);for(int i=0;i<8;i++)tick(io,one);
            check(one.getState()==TeamOnePollenAuto.State.WAITING&&io.count==0,"Timeout holds still without inventing collected balls");wheelsOff(io);
            io=new IO();one=garden(io,f,a,5);io.count=4;tick(io,one);wheelsOff(io);check(io.out.intakePower==0,"Full sensor stops forward stroke and intake immediately");
            io=new IO();io.pose=f.teamTwoWallHive[a];io.count=4;io.targets=tags(f,a,true);
            TeamTwoHiveAuto two=new TeamTwoHiveAuto(io,f);two.spinUpSeconds=0;two.feedSeconds=.08;two.start(a,.5);
            for(int i=0;i<80&&two.getState()!=TeamTwoHiveAuto.State.TO_FLOWER;i++){tick(io,two);if(io.out.feederPower>0)io.count=0;}
            for(int i=0;i<40&&two.getState()!=TeamTwoHiveAuto.State.INTAKE;i++){
                if(two.destination()!=null)io.pose=two.destination();
                if(two.getState()==TeamTwoHiveAuto.State.PRESS_FLOWER)io.time+=.42;
                tick(io,two);
            }
            check(two.getState()==TeamTwoHiveAuto.State.INTAKE,"Reach Flower pickup");
            io.targets=new Target[]{new Target(-1,1.5,0,Double.NaN,5,"yellow")};tick(io,two);
            check(io.out.turnClockwise==0&&io.out.strafeRight==0,"Team Two ignores small camera jitter instead of turning");
            io.targets=new Target[0];tick(io,two);check(two.getState()==TeamTwoHiveAuto.State.WAITING,"Lost pollen target waits instead of spinning");wheelsOff(io);
            io.time=25.9;two.tick();check(two.getState()==TeamTwoHiveAuto.State.WAITING,"Team Two stationary wait until parking trigger");wheelsOff(io);
            io.time=26;two.tick();check(two.getState()==TeamTwoHiveAuto.State.PARK&&io.out.intakePower==0,"Parking overrides collection motor");
        }
        System.out.println("Both alliances: bounded Garden forward stroke, intake/full/timeout/park stops, camera deadbands and stationary 26-second waits passed.");
    }
    private static TeamOnePollenAuto windowGarden(IO io,FieldLayout f,int alliance){
        io.pose=f.hive[alliance];io.targets=tags(f,alliance,false);
        TeamOnePollenAuto auto=new TeamOnePollenAuto(io,f);
        auto.gardenYellowWindowEnabled=true;auto.translationPowerMultiplier=1.25;
        auto.spinUpSeconds=0;auto.feedSeconds=.08;auto.start(alliance,.5);
        for(int i=0;i<30&&auto.getState()!=TeamOnePollenAuto.State.INTAKE_GARDEN;i++){
            if(auto.destination()!=null)io.pose=auto.destination();
            tick(io,auto);
        }
        check(auto.getState()==TeamOnePollenAuto.State.INTAKE_GARDEN,"Reach actual Garden intake sequence");
        io.targets=new Target[]{new Target(-1,0,0,Double.NaN,5,"yellow")};return auto;
    }
    private static void yellowWindowChecks(){
        for(int alliance=0;alliance<2;alliance++){
            FieldLayout f=new FieldLayout();IO io=new IO();TeamOnePollenAuto auto=windowGarden(io,f,alliance);
            io.time=9.999;io.targets=new Target[0];auto.tick();
            check(!auto.leaveAuthorized(),"No absence authorization before second 10");
            io.time=10;auto.tick();
            check(auto.leaveAuthorized()&&auto.getState()==TeamOnePollenAuto.State.WAITING,"Fresh no-target frame authorizes leaving at second 10");
            wheelsOff(io);check(io.out.intakePower==0,"Empty Garden stops intake without inventing counts");
            io.time=25.9;auto.tick();check(auto.getState()==TeamOnePollenAuto.State.WAITING,"Preserve regular routine's parking schedule");
            io.time=26;auto.tick();check(auto.getState()==TeamOnePollenAuto.State.PARK,"Regular parking still starts at 26 seconds");

            io=new IO();auto=windowGarden(io,f,alliance);
            for(double time:new double[]{10,14,18}){
                io.time=time;auto.tick();
                check(!auto.leaveAuthorized()&&auto.getState()==TeamOnePollenAuto.State.INTAKE_GARDEN,"Yellow keeps forward collection through window boundaries");
                check(Math.abs(io.out.forward-auto.gardenAdvancePower*1.25)<1e-9&&io.out.intakePower==1,"Translation is 1.25x with unchanged intake");
            }
            io.time=18.001;io.targets=new Target[0];auto.tick();
            check(!auto.leaveAuthorized(),"No new absence authorization after second 18");

            io=new IO();auto=windowGarden(io,f,alliance);io.time=12;io.targets=new Target[0];
            io.connected=false;auto.tick();check(!auto.leaveAuthorized(),"Disconnected camera cannot prove empty Garden");wheelsOff(io);
            io.connected=true;io.frameAvailable=false;auto.tick();check(!auto.leaveAuthorized(),"Missing frame cannot authorize leaving");
            io.frameAvailable=true;io.frameAge=.251;auto.tick();check(!auto.leaveAuthorized(),"Stale frame cannot authorize leaving");
            io.frameAge=Double.NaN;auto.tick();check(!auto.leaveAuthorized(),"Invalid timestamp cannot authorize leaving");
            io.frameAge=0;io.targets=new Target[]{new Target(-1,0,0,Double.NaN,5,"yellow")};auto.tick();
            check(io.out.forward>0&&!auto.leaveAuthorized(),"Fresh yellow resumes collection after camera recovery");
            io.targets=new Target[0];io.time=12.1;auto.tick();check(auto.leaveAuthorized(),"Yellow disappearance authorizes leaving immediately");

            io=new IO();auto=windowGarden(io,f,alliance);auto.useTeamTwoFollowUpShot=true;
            io.time=12;io.targets=new Target[0];auto.tick();check(!auto.leaveAuthorized(),"Backup is excluded from yellow-window departure");
            io=new IO();auto=windowGarden(io,f,alliance);io.time=12;io.count=4;auto.tick();
            check(auto.getState()==TeamOnePollenAuto.State.PARK&&io.out.intakePower==0,"Measured full inventory preserves existing exit");
            io=new IO();auto=windowGarden(io,f,alliance);io.time=30;auto.tick();
            check(!auto.isRunning(),"Hard stop remains at 30 seconds");wheelsOff(io);
        }
        System.out.println("Team One yellow window: boundaries, absence, fresh recovery, backup exclusion, 1.25x translation and parking/stop checks passed.");
    }
}
