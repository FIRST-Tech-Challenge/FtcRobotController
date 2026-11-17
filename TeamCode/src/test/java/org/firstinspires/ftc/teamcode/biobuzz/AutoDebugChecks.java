package org.firstinspires.ftc.teamcode.biobuzz;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Telemetry formatting is read-only and distinguishes raw, remapped, stale and unavailable data. */
public final class AutoDebugChecks {
    private static void check(boolean ok,String why){if(!ok)throw new AssertionError(why);}
    public static void main(String[] args){
        double now=2;
        Vision raw=new Vision(true,0,now,new Target[]{new Target(42,1,-3),new Target(43,-1,-4,52.02),new Target(42,1,-3)});
        check(AutoDebugSnapshot.ids(raw,now).equals("[42, 43]"),"Sorted unique raw camera IDs");
        check(AutoDebugSnapshot.details(raw,now).contains("UNCALIBRATED"),"Missing height calibration is explicit");
        check(AutoDebugSnapshot.ids(raw,now+.5).equals("No fresh AprilTags"),"Do not present stale detections as current");
        check(AutoDebugSnapshot.age(raw,now+.5).contains("STALE"),"Show stale-frame age");
        check(AutoDebugSnapshot.age(raw,0).equals("Invalid timestamp"),"Clock reset must not display a negative valid age");
        Vision color=new Vision(true,1,now,new Target[]{new Target(-1,0,0)});
        check(AutoDebugSnapshot.ids(color,now).contains("not sampled"),"Color pipeline is not incorrectly reported as a missing tag");
        check(AutoDebugSnapshot.ids(null,now).contains("Waiting"),"Before first observation is explicit");
        Pose pose=new Pose(-12.75,-62.9,-Math.PI/2);
        Sensors valid=new Sensors(pose,now,true,true,4,raw);
        String shown=AutoDebugSnapshot.pose(valid);
        check(shown.contains("-12.8 in")&&shown.contains("-62.9 in")&&shown.contains("-90.0 deg"),"Position in inches and heading in degrees");
        check(AutoDebugSnapshot.pose(new Sensors(pose,now,false,true,-1,raw)).contains("INVALID"),"Cached invalid pose cannot appear current");
        check(AutoDebugSnapshot.pose(null).equals("UNAVAILABLE"),"No sample is not a zero location");
        Vision canonical=new Vision(true,0,now,new Target[]{new Target(34,0,0),new Target(35,0,0)});
        check(!AutoDebugSnapshot.ids(raw,now).equals(AutoDebugSnapshot.ids(canonical,now)),"Keep camera IDs distinct from canonical aliases");
        check(raw.targets[0].id==42&&raw.targets.length==3,"Rendering must not mutate observations");
        System.out.println("Debug telemetry: raw/remapped IDs, deduplication, stale/color/no-frame messages, calibration and pose validity passed.");
    }
}
