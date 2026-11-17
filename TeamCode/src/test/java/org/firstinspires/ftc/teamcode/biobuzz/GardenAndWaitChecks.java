package org.firstinspires.ftc.teamcode.biobuzz;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Injected observations verify decisions, not physical intake travel or odometry calibration. */
public final class GardenAndWaitChecks {
    static final class IO implements RobotIO {
        double time;int count=0,pipeline;Pose pose;Target[] targets={};Outputs out=new Outputs();boolean stop;
        public double nowSeconds(){return time;}
        public boolean stopRequested(){return stop;}
        public Sensors readSensors(){return new Sensors(pose,time,true,true,count,count,new Vision(true,pipeline,time,targets));}
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
}
