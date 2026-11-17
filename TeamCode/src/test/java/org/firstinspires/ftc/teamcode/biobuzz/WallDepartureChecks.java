package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Hardware mounting regression; not a physical wiring/odometry calibration test. */
public final class WallDepartureChecks {
    private static void check(boolean ok,String why){if(!ok)throw new AssertionError(why);}
    private static void near(double a,double b,String why){check(Math.abs(a-b)<1e-9,why);}
    private static double forward(double[] w){return (w[0]+w[1]+w[2]+w[3])/4;}
    private static double strafe(double[] w){return (w[0]-w[1]-w[2]+w[3])/4;}
    private static double turn(double[] w){return (w[0]-w[1]+w[2]-w[3])/4;}
    private static final class IO implements RobotIO {
        final FieldLayout field=new FieldLayout();final int alliance;
        double time;int total=4,pollen=4;Pose pose;Outputs out=new Outputs();
        IO(int alliance,boolean two){this.alliance=alliance;pose=two?field.teamTwoWallHive[alliance]:field.hive[alliance];}
        public double nowSeconds(){return time;}
        public Sensors readSensors(){
            int[] ids=field.farTags[alliance];
            Vision v=new Vision(true,out.pipeline,time,new Target[]{new Target(ids[0],0,0,field.upperTagHeight),new Target(ids[1],0,0,field.upperTagHeight)});
            return new Sensors(pose,time,true,true,total,pollen,v);
        }
        public void apply(Outputs o){out=o.copy();}
    }
    private static void departure(int alliance,boolean two){
        IO io=new IO(alliance,two);
        RobotController auto=two?new TeamTwoHiveAuto(io,io.field):new TeamOnePollenAuto(io,io.field);
        auto.start(alliance,.5);
        boolean sawFeed=false,found=false;
        for(int i=0;i<700&&auto.isRunning();i++){
            auto.tick();
            // Explicit fixture observation of discharged preloads; never production counting by time.
            if(io.out.feederPower>0){sawFeed=true;io.total=io.pollen=0;}
            Outputs o=io.out;
            if(Math.abs(o.forward)+Math.abs(o.strafeRight)+Math.abs(o.turnClockwise)>1e-6){
                check(sawFeed,"Do not move before the preload attempt");
                check(o.forward<0,"First post-shot command must back away from the wall");
                near(o.strafeRight,0,"Retreat longitudinally before crossing/turning near wall");
                near(o.turnClockwise,0,"Retreat at the wall-facing start heading");
                double[] hardware=o.wheelPowers(-1);
                double physicalForward=-forward(hardware); // Observed robot's mounting.
                near(physicalForward,o.forward,"Hardware correction restores requested body direction");
                double dt=.02,h=io.pose.heading;
                Pose after=new Pose(io.pose.x+(physicalForward*Math.cos(h)+strafe(hardware)*Math.sin(h))*25*dt,
                    io.pose.y+(physicalForward*Math.sin(h)-strafe(hardware)*Math.cos(h))*25*dt,h);
                check(Math.abs(after.y)<Math.abs(io.pose.y),"Actual corrected move increases wall clearance");
                check(io.field.allowed(after),"Retreat stays in the field");
                double uncorrected=-forward(o.wheelPowers());
                check(Math.abs(io.pose.y+uncorrected*Math.sin(h)*25*dt)>Math.abs(io.pose.y),
                    "Old uncorrected hardware output reproduces the drive-into-wall intent");
                Pose wallPose=io.pose;io.pose=after;io.time+=dt;
                for(int j=0;j<100&&io.pose.distance(wallPose)<5.4;j++){
                    auto.tick();Outputs back=io.out;
                    check(back.forward<0,"Keep backing until wall clearance is reached");
                    near(back.strafeRight,0,"No sideways movement during wall retreat");
                    near(back.turnClockwise,0,"No turn during wall retreat");
                    double[] w=back.wheelPowers(-1);double physical=-forward(w);
                    io.pose=new Pose(io.pose.x+physical*Math.cos(h)*25*dt,io.pose.y+physical*Math.sin(h)*25*dt,h);
                    check(io.field.allowed(io.pose),"Entire measured retreat stays collision-clear");io.time+=dt;
                }
                check(io.pose.distance(wallPose)>=5.4,"Complete six-inch retreat within arrival tolerance");
                found=true;break;
            }
            io.time+=.02;
        }
        check(found,"Find post-shot retreat: alliance="+alliance+" teamTwo="+two+" status="+auto.status());
        auto.stop();near(io.out.forward,0,"STOP clears retreat command");
        System.out.println((alliance==0?"Red":"Blue")+" Team "+(two?"Two":"One")+": first post-shot hardware movement backs away at the correct heading");
    }
    public static void main(String[] args){
        Outputs o=new Outputs();o.forward=.3;o.strafeRight=.2;o.turnClockwise=.1;o.intakePower=.7;
        double[] raw=o.wheelPowers(),fixed=o.wheelPowers(-1);
        near(forward(raw),.3,"Ideal/simulator mixer unchanged");
        near(-forward(fixed),.3,"Physical forward matches intent");
        near(strafe(fixed),.2,"Strafe not inverted");
        near(turn(fixed),.1,"Turn not inverted");
        near(o.forward,.3,"Mapping does not mutate controller command");near(o.intakePower,.7,"Mechanisms unchanged");
        for(double bad:new double[]{0,.5,Double.NaN}){boolean rejected=false;try{o.wheelPowers(bad);}catch(IllegalArgumentException expected){rejected=true;}check(rejected,"Reject invalid hardware direction calibration");}
        for(int a=0;a<2;a++){departure(a,false);departure(a,true);}
        System.out.println("Wall-facing Red/Blue starts, actual mounting, retreat direction, unchanged other axes and STOP checks passed.");
    }
}
