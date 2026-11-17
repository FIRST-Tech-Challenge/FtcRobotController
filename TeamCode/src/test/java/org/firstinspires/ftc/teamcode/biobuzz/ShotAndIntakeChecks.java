package org.firstinspires.ftc.teamcode.biobuzz;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Injected sensor checks; not a physical robot or camera calibration. */
public final class ShotAndIntakeChecks {
    static void check(boolean ok,String why){if(!ok)throw new AssertionError(why);}
    static final class IO implements RobotIO {
        double time,age;boolean connected=true,valid=true;int count=4,pollen=4,pipeline;
        Pose pose;Target[] tags={};Outputs out=new Outputs();
        public double nowSeconds(){return time;}
        public Sensors readSensors(){return new Sensors(pose,time,true,connected,count,pollen,new Vision(valid,pipeline,time-age,tags));}
        public void apply(Outputs o){out=o.copy();pipeline=o.pipeline;}
    }
    static Target[] tags(FieldLayout f,int a,boolean flipped){int[] ids=flipped?f.farTags[a]:f.nearTags[a];return new Target[]{new Target(ids[0],0,0,f.upperTagHeight),new Target(ids[1],0,0,f.upperTagHeight)};}
    static void tick(IO io,RobotController auto){io.time+=.04;auto.tick();}
    static void noShot(IO io){check(io.out.feederPower==0&&io.out.shooterPower==0,"Unauthorized shot outputs must be OFF");}
    public static void main(String[] args){
        for(int a=0;a<2;a++)for(boolean two:new boolean[]{false,true}){
            FieldLayout f=new FieldLayout();IO io=new IO();io.pose=two?f.teamTwoWallHive[a]:f.hive[a];
            if(two){
                check(f.roleStart(a,true).distance(io.pose)==0,"Team Two initializes odometry directly at its shooting pose");
                check(Math.abs(Pose.angle(f.roleStart(a,true).heading-io.pose.heading))<1e-9,"Team Two rear shooter has the correct start heading");
            }
            RobotController auto=two?new TeamTwoHiveAuto(io,f):new TeamOnePollenAuto(io,f);
            auto.start(a,.5);
            for(int i=0;i<40;i++){tick(io,auto);noShot(io);check(io.out.forward==0&&io.out.strafeRight==0&&io.out.turnClockwise==0,"At-shot startup waits stationary for tags");}
            io.tags=tags(f,1-a,two);for(int i=0;i<12;i++){tick(io,auto);noShot(io);}
            io.tags=tags(f,a,!two);for(int i=0;i<12;i++){tick(io,auto);noShot(io);}
            io.tags=tags(f,a,two);io.age=.5;for(int i=0;i<12;i++){tick(io,auto);noShot(io);}io.age=0;
            io.connected=false;for(int i=0;i<12;i++){tick(io,auto);noShot(io);}io.connected=true;
            boolean feeding=false;for(int i=0;i<60;i++){tick(io,auto);if(io.out.feederPower>0){feeding=true;break;}}
            check(feeding,"Fresh correct raised tags authorize feeding");
            io.valid=false;tick(io,auto);noShot(io);io.valid=true;
            io.count=io.pollen=4;
            for(int i=0;i<220;i++){
                tick(io,auto);
                if(two&&io.out.feederPower>0){io.count=io.pollen=0;}
                if((two&&auto.getState()==TeamTwoHiveAuto.State.TO_FLOWER)||(!two&&auto.getState()==TeamOnePollenAuto.State.TO_GARDEN))break;
            }
            check(two?auto.getState()==TeamTwoHiveAuto.State.TO_FLOWER:auto.getState()==TeamOnePollenAuto.State.TO_GARDEN,"Proceed to the role pickup");
            io.count=io.pollen=0;tick(io,auto);check(io.out.intakePower==1,"Intake starts while driving to pickup");
            io.count=io.pollen=3;tick(io,auto);check(io.out.intakePower==1,"Three pollen do not stop intake");
            io.count=io.pollen=-1;tick(io,auto);check(io.out.intakePower==1,"Unknown inventory is not fabricated as full");
            io.count=io.pollen=4;tick(io,auto);check(io.out.intakePower==0,"Four measured pieces stop intake");
            io.time=26;auto.tick();check(io.out.intakePower==0&&io.out.feederPower==0&&io.out.shooterPower==0,"Parking cancels all mechanisms");
            if(two){Pose p=auto.destination();check(p.y==(a==0?51:-51),"Team Two parks in the shooting-side half");}
        }
        FieldLayout f=new FieldLayout();IO io=new IO();io.pose=f.hive[0];
        for(Target[] bad:new Target[][]{{new Target(34,0,0,52.02),new Target(34,0,0,52.02)}, {new Target(34,15,0,52.02),new Target(35,15,0,52.02)}, {new Target(34,0,0,36.57),new Target(35,0,0,36.57)}}){
            HiveShotGate gate=new HiveShotGate(f,0,false);io.tags=bad;
            for(int i=0;i<8;i++){io.time+=.04;check(!gate.ready(io.readSensors(),io.time),"Duplicate, off-axis or lowered tags cannot authorize feed");}
        }
        MirroredHiveTags alias=new MirroredHiveTags(f,false);io.tags=tags(f,1,false);
        Vision v=alias.normalize(io.readSensors().vision,io.time,true);
        check(alias.observedFamily()==1&&v.targets[0].id==f.nearTags[0][0],"Team One aliases Blue near tags without an alliance selector");
        System.out.println("Both hardware roles and alliances: tag rejection, loss shutdown, intake en route/full/park, shooting-side parking and near-tag aliases passed.");
    }
}
