package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Target;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Vision;

/** Alliance filtering and stable Hive-state observation, shared by both adapters.
 * Tag IDs identify a Cell; measured height distinguishes its stable up/down pose.
 * No shot is authorized by an uncalibrated, stale, contradictory or tipping frame. */
public final class HiveVision {
    private final FieldLayout field;
    private final int alliance;
    private Vision latest;
    private double lastFrame=Double.NEGATIVE_INFINITY,candidateAt,candidateResidual;
    private int candidate=-1,confirmed=-1,frames;
    public HiveVision(FieldLayout field,int alliance) {
        if(alliance<0||alliance>1)throw new IllegalArgumentException("Alliance must be 0 or 1");
        this.field=field;this.alliance=alliance;
    }
    public void update(Vision vision,double now) {
        if(vision==null||!vision.fresh(now,0)) { latest=vision;resetCandidate();return; }
        if(vision.capturedAt<lastFrame){latest=null;resetCandidate();return;}
        if(vision.capturedAt==lastFrame) return; // repeated frames cannot confirm a tip
        latest=vision;
        lastFrame=vision.capturedAt;
        int near=cellHeightState(vision,field.nearTags[alliance]);
        int far=cellHeightState(vision,field.farTags[alliance]);
        // 1 means this Cell is up, 0 means down; -1 means no reliable observation.
        int observed=near==1||far==0?0:near==0||far==1?1:-1;
        if((near>=0&&far>=0&&near==far)||near==-2||far==-2) observed=-1;
        if(observed<0) { resetCandidate();return; }
        int[] evidence=near>=0?field.nearTags[alliance]:field.farTags[alliance];
        double sum=0;int n=0;
        for(Target t:vision.targets)if(contains(evidence,t.id)&&Double.isFinite(t.heightInches)){sum+=t.heightInches;n++;}
        int evidenceState=near>=0?near:far;
        double residual=sum/n-(evidenceState==1?field.upperTagHeight:field.lowerTagHeight);
        if(observed!=candidate||Math.abs(residual-candidateResidual)>field.tagHeightMotionTolerance) {
            candidate=observed;candidateAt=vision.capturedAt;candidateResidual=residual;frames=1;confirmed=-1;
        }
        else frames++;
        if(frames>=3&&vision.capturedAt-candidateAt>=.10)confirmed=candidate;
    }
    private void resetCandidate() { candidate=confirmed=-1;frames=0; }
    /** -1 means unknown or in motion; 0 normal, 1 opposite Cell up. */
    public int state(double now) {
        return latest!=null&&latest.fresh(now,0)?confirmed:-1;
    }
    public Target aim(double now,boolean flipped) {
        int expected=flipped?1:0;
        if(state(now)!=expected)return null;
        int[] ids=flipped?field.farTags[alliance]:field.nearTags[alliance];
        double tx=0,ty=0,height=0;int n=0,id=-1;
        for(Target t:latest.targets)if(contains(ids,t.id)&&Double.isFinite(t.tx)&&Double.isFinite(t.ty)
                &&Math.abs(t.heightInches-field.upperTagHeight)<=field.tagHeightTolerance){
            tx+=t.tx;ty+=t.ty;height+=t.heightInches;n++;id=t.id;
        }
        return n==0?null:new Target(id,tx/n,ty/n,height/n);
    }
    private int cellHeightState(Vision v,int[] ids) {
        int up=0,down=0,bad=0;
        for(Target t:v.targets)if(contains(ids,t.id)&&Double.isFinite(t.heightInches)) {
            if(Math.abs(t.heightInches-field.upperTagHeight)<=field.tagHeightTolerance)up++;
            else if(Math.abs(t.heightInches-field.lowerTagHeight)<=field.tagHeightTolerance)down++;
            else bad++;
        }
        if(bad>0||(up>0&&down>0))return -2;
        return up>=2?1:down>=2?0:-1;
    }
    private static boolean contains(int[] ids,int id) {for(int i:ids)if(i==id)return true;return false;}
}
