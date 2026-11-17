package org.firstinspires.ftc.teamcode.biobuzz;
import java.util.*;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;
/** Unique own raised-Cell tags, three fresh stable frames and centered aim. */
public final class HiveShotGate {
    private final HiveVision vision;
    private final int[] ids;
    private final boolean flipped;
    public HiveShotGate(FieldLayout field,int alliance,boolean flipped){
        vision=new HiveVision(field,alliance);this.flipped=flipped;
        ids=flipped?field.farTags[alliance]:field.nearTags[alliance];
    }
    public boolean ready(Sensors s,double now){
        List<Target> selected=new ArrayList<Target>();Set<Integer> seen=new HashSet<Integer>();
        if(s.cameraConnected)for(Target t:s.vision.targets)for(int id:ids)
            if(t.id==id&&Double.isFinite(t.tx)&&Math.abs(t.tx)<=10&&Double.isFinite(t.ty)&&seen.add(id)){selected.add(t);break;}
        vision.update(new Vision(s.cameraConnected&&s.vision.valid&&selected.size()>=2,s.vision.pipeline,s.vision.capturedAt,selected.toArray(new Target[0])),now);
        Target aim=vision.aim(now,flipped);return aim!=null&&Math.abs(aim.tx)<=2;
    }
}
