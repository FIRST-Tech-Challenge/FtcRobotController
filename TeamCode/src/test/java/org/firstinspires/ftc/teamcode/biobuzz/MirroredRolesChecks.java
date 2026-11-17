package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Synthetic observations and mirrored hardware-motion model, not physical validation. */
public final class MirroredRolesChecks {
    private static void check(boolean b,String why){if(!b)throw new AssertionError(why);}
    private static void near(double a,double b,String why){check(Math.abs(a-b)<1e-7,why);}
    private static Vision tags(FieldLayout f,int family,double time,double height){int[] ids=f.farTags[family];return new Vision(true,0,time,new Target[]{new Target(ids[0],3,-2,height),new Target(ids[1],-3,2,height)});}
    private static void aliasChecks(){
        FieldLayout f=new FieldLayout();MirroredHiveTags m=new MirroredHiveTags(f);
        check(!m.normalize(tags(f,0,0,52.02),0,false).valid&&m.observedFamily()==-1,"Do not choose Hive in transit");
        check(!m.normalize(tags(f,0,.1,36.57),.1,true).valid,"Unraised far Cell cannot identify the shot Hive");
        int id=f.farTags[0][0];Vision duplicate=new Vision(true,0,.2,new Target[]{new Target(id,0,0,52.02),new Target(id,0,0,52.02)});
        check(!m.normalize(duplicate,.2,true).valid,"Duplicate ID is not two distinct tags");
        Vision r=tags(f,0,.3,52.02),b=tags(f,1,.3,52.02);
        Vision mixed=new Vision(true,0,.3,new Target[]{r.targets[0],r.targets[1],b.targets[0],b.targets[1]});
        check(!m.normalize(mixed,.3,true).valid&&m.observedFamily()==-1,"Two Hives are ambiguous without manual selection");
        MirroredHiveTags centered=new MirroredHiveTags(f);
        Vision adjacent=new Vision(true,0,.4,new Target[]{b.targets[0],b.targets[1],new Target(f.farTags[0][0],25,0,52.02),new Target(f.farTags[0][1],28,0,52.02)});
        check(centered.normalize(adjacent,.4,true).valid&&centered.observedFamily()==1,"Centered own Hive is distinguishable from adjacent off-axis Hive");
        b=tags(f,1,1,52.02);Vision v=m.normalize(b,1,true);
        check(m.observedFamily()==1&&v.valid,"Recognize Blue family automatically at shot pose");
        check(v.targets[0].id==f.farTags[0][0]&&b.targets[0].id==f.farTags[1][0],"Alias without mutating raw observations");
        near(v.capturedAt,b.capturedAt,"Preserve capture time");near(v.targets[0].tx,b.targets[0].tx,"Preserve image angle");near(v.targets[0].heightInches,52.02,"Preserve measured height");
        HiveVision gate=new HiveVision(f,0);gate.update(v,1);
        for(int i=1;i<=4;i++){gate.update(m.normalize(b,1+i*.04,true),1+i*.04);check(gate.state(1+i*.04)==-1,"Aliasing cannot make repeated frames fresh");}
        gate.update(m.normalize(tags(f,1,1.20,52.02),1.20,true),1.20);
        gate.update(m.normalize(tags(f,1,1.26,52.02),1.26,true),1.26);
        check(gate.state(1.26)==1,"Three distinct stable raised frames still required");
        check(!m.normalize(tags(f,0,1.3,52.02),1.3,true).valid&&m.observedFamily()==1,"Never switch to the other Hive after lock");
        check(!m.normalize(tags(f,1,1.4,52.02),2,true).valid,"Reject stale frames");
        Vision color=new Vision(true,1,2,new Target[]{new Target(-1,0,0,Double.NaN,2,"yellow")});check(m.normalize(color,2,false)==color,"Floor targets unchanged");
        m.reset();check(m.observedFamily()==-1,"New run clears detected family");m.normalize(tags(f,0,3,52.02),3,true);
        check(!m.normalize(tags(f,0,2.99,52.02),3,true).valid,"Reject backward capture timestamps");
    }
    private static void geometry(boolean two){
        FieldLayout f=new FieldLayout();Pose red=f.roleStart(0,two),blue=f.roleStart(1,two);
        Pose[][] sites={f.hive,f.garden,f.flower,f.teamTwoWallHive,f.teamTwoFlowerTouch,f.teamTwoFlowerClear,f.teamTwoFlowerLane,f.teamTwoFlowerPickup,f.teamOneStartSidePark,f.teamTwoGardenSidePark};
        for(Pose[] site:sites){
            double rx=site[0].x-red.x,ry=site[0].y-red.y,bx=site[1].x-blue.x,by=site[1].y-blue.y;
            near(rx*Math.cos(red.heading)+ry*Math.sin(red.heading),bx*Math.cos(blue.heading)+by*Math.sin(blue.heading),"Same body-relative forward distance on both alliances");
            near(rx*Math.sin(red.heading)-ry*Math.cos(red.heading),bx*Math.sin(blue.heading)-by*Math.cos(blue.heading),"Same body-relative strafe distance on both alliances");
            near(Pose.angle((site[0].heading-red.heading)-(site[1].heading-blue.heading)),0,"Same heading change on either side");
        }
    }
    private static final class IO implements RobotIO {
        final FieldLayout field=new FieldLayout();final MirroredHiveTags tags=new MirroredHiveTags(field);
        final int side;final boolean two;Pose physical;double time;int count=4,pollen=4;Outputs out=new Outputs();
        IO(int side,boolean two){this.side=side;this.two=two;physical=field.roleStart(side,two);}
        Pose local(){return side==0?physical:new Pose(-physical.x,-physical.y,Pose.angle(physical.heading-Math.PI));}
        public double nowSeconds(){return time;}
        public Sensors readSensors(){
            Pose p=local(),shot=field.teamTwoWallHive[0];
            Vision v=out.pipeline==0?MirroredRolesChecks.tags(field,side,time,52.02):new Vision(true,1,time,new Target[]{new Target(-1,0,-10,Double.NaN,2,"yellow")});
            if(two)v=tags.normalize(v,time,p.distance(shot)<=1&&Math.abs(Pose.angle(p.heading-shot.heading))<=Math.toRadians(2));
            return new Sensors(p,time,true,true,count,pollen,v);
        }
        public void apply(Outputs o){out=o.copy();}
    }
    private static void routine(int side,boolean two){
        IO io=new IO(side,two);RobotController auto=two?new TeamTwoHiveAuto(io,io.field):new TeamOnePollenAuto(io,io.field);auto.start(0,.5);
        int fed=0,intake=0;double lastShot=-1;boolean pickup=false;
        for(int n=0;n<1501&&auto.isRunning();n++){
            auto.tick();String state=auto.getState().name();Outputs o=io.out;
            // Explicit fixture inventory events. Production does not infer counts from timers.
            if(o.feederPower>0&&io.count>0&&io.time-lastShot>=.68){io.count--;io.pollen--;fed++;lastShot=io.time;}
            if(state.equals(two?"INTAKE":"INTAKE_GARDEN")){pickup=true;if(++intake>=20)io.count=io.pollen=4;}
            double[] w=o.wheelPowers(-1);double forward=-(w[0]+w[1]+w[2]+w[3])/4,strafe=(w[0]-w[1]-w[2]+w[3])/4,turn=(w[0]-w[1]+w[2]-w[3])/4;
            double h=Pose.angle(io.physical.heading-turn*1.65*.02);
            Pose next=new Pose(io.physical.x+(forward*Math.cos(h)+strafe*Math.sin(h))*25*.02,io.physical.y+(forward*Math.sin(h)-strafe*Math.cos(h))*25*.02,h);
            if(state.equals("PRESS_FLOWER")&&!io.field.allowed(next))next=io.physical; // Physical trigger post stops the bounded contact.
            check(io.field.allowed(next),"Mirrored route collision: side="+side+" state="+state);io.physical=next;io.time+=.02;
        }
        Pose park=(two?io.field.teamTwoGardenSidePark:io.field.teamOneStartSidePark)[side];
        check(!auto.isRunning()&&io.time<30&&io.physical.distance(park)<=2,"No-selection mirrored routine parks before 30 s: "+auto.status());
        check(pickup&&fed==(two?8:4),"Both roles retain pickup and intended feed batches");
        if(two)check(io.tags.observedFamily()==side,"Automatically observe the physical Hive family, not a driver color input");
        System.out.println("No-selection Team "+(two?2:1)+" on "+(side==0?"Red":"Blue")+": "+fed+" observed fixture feeds, parked at "+io.time);
    }
    public static void main(String[] args){aliasChecks();for(boolean two:new boolean[]{false,true}){geometry(two);routine(0,two);routine(1,two);}System.out.println("No gamepad/alliance selection, 180-degree geometry, camera-family lock, frame safety and mirrored hardware routines passed.");}
}
