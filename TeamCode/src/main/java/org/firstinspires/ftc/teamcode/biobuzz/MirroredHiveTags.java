package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Alias measured Red/Blue tag IDs into one 180-degree-symmetric route frame.
 * No alliance input. Lock one visible Hive family only at the wall shot pose.
 * Existing controller still requires three fresh stable frames before feeding. */
public final class MirroredHiveTags {
    public static final double MAX_CENTER_TX_DEGREES=10;
    private final FieldLayout field;
    private final boolean flipped;
    private int family=-1;
    private double lastFrame=Double.NEGATIVE_INFINITY;
    public MirroredHiveTags(FieldLayout field){this(field,true);}
    public MirroredHiveTags(FieldLayout field,boolean flipped){this.field=field;this.flipped=flipped;}
    public void reset(){family=-1;lastFrame=Double.NEGATIVE_INFINITY;}
    public int observedFamily(){return family;}
    public Vision normalize(Vision v,double now,boolean atShot){
        if(v.pipeline!=0)return v; // Floor-color measurements have no alliance aliases.
        if(!v.fresh(now,0)||v.capturedAt<lastFrame)return Vision.empty(0,now);
        lastFrame=v.capturedAt;
        if(family<0){
            if(!atShot)return Vision.empty(0,now);
            int[] raised=new int[2];
            for(int a=0;a<2;a++){
                for(int id:(flipped?field.farTags[a]:field.nearTags[a]))for(Target t:v.targets)if(t.id==id&&Double.isFinite(t.tx)&&Math.abs(t.tx)<=MAX_CENTER_TX_DEGREES&&Double.isFinite(t.ty)
                        &&Double.isFinite(t.heightInches)&&Math.abs(t.heightInches-field.upperTagHeight)<=field.tagHeightTolerance){raised[a]++;break;}
            }
            // The adjacent Hive may be in the wide FOV: require one centered raised
            // pair at the known shot pose. Two centered pairs remain ambiguous.
            if((raised[0]>=2)==(raised[1]>=2))return Vision.empty(0,now);
            int candidate=raised[0]>=2?0:1;
            family=candidate;
        }
        List<Target> mapped=new ArrayList<Target>();
        for(Target t:v.targets){
            int id=alias(t.id);
            if(id>=0)mapped.add(new Target(id,t.tx,t.ty,t.heightInches,t.areaPercent,t.color));
        }
        // Preserve real timestamp, measured heights and image angles. Opposite family is ignored.
        return new Vision(v.valid&&!mapped.isEmpty(),0,v.capturedAt,mapped.toArray(new Target[0]));
    }
    private int alias(int id){
        for(int i=0;i<field.nearTags[family].length;i++)if(id==field.nearTags[family][i])return field.nearTags[0][i];
        for(int i=0;i<field.farTags[family].length;i++)if(id==field.farTags[family][i])return field.farTags[0][i];
        return -1;
    }
}
