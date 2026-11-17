package org.firstinspires.ftc.teamcode.quantumstorm.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;

/** Actual SDK observations; never invents detections, tilt state or inventory. */
public final class TeamOneLimelight implements AutoCloseable {
    private final Limelight3A camera;
    private final Servo tilt;
    private final DoubleSupplier clock;
    private final boolean teamTwo;
    private final String hardwareName;
    private boolean lastConnected;
    private int pipeline=-1;
    private double changedAt,frameTimestamp=Double.NaN,capturedAt;
    private String problem="";
    public TeamOneLimelight(HardwareMap map,DoubleSupplier clock){this(map,clock,false);}
    /** Team Two shares camera transport but has separate height calibration and shot gating. */
    public TeamOneLimelight(HardwareMap map,DoubleSupplier clock,boolean teamTwo){
        this.clock=clock;this.teamTwo=teamTwo;
        Limelight3A found=map.tryGet(Limelight3A.class,Constants.LIMELIGHT);
        String name=Constants.LIMELIGHT;
        // Preserve support for the hardware name used by the current approach OpMode.
        if(found==null){found=map.tryGet(Limelight3A.class,"limelight 3A");name="limelight 3A";}
        hardwareName=found==null?"Not found (tried "+Constants.LIMELIGHT+", limelight 3A)":name;
        camera=found;tilt=map.tryGet(Servo.class,Constants.LIMELIGHT_TILT_SERVO);
        if(camera!=null){try{camera.start();reset();}catch(RuntimeException e){problem=e.getMessage();}}
    }
    public void reset(){pipeline=-1;select(0);}
    public void select(int mode){
        if(Thread.currentThread().isInterrupted())throw new java.util.concurrent.CancellationException("STOP");
        if(mode==pipeline)return;
        pipeline=mode;changedAt=clock.getAsDouble();frameTimestamp=Double.NaN;
        if(camera==null)return;
        try{
            if(tilt!=null)tilt.setPosition(mode==0?Constants.LIMELIGHT_TILT_UP:Constants.LIMELIGHT_TILT_DOWN);
            camera.pipelineSwitch(mode==0?Constants.LIMELIGHT_APRILTAG_PIPELINE:Constants.LIMELIGHT_COLOR_PIPELINE);
        }catch(RuntimeException e){problem=e.getMessage();}
    }
    public boolean mapped(){return camera!=null;}
    public String hardwareName(){return hardwareName;}
    public int selectedPipeline(){return pipeline==0?Constants.LIMELIGHT_APRILTAG_PIPELINE:Constants.LIMELIGHT_COLOR_PIPELINE;}
    public boolean connected(){try{lastConnected=camera!=null&&camera.isConnected();}catch(RuntimeException e){problem=e.getMessage();lastConnected=false;}return lastConnected;}
    public String cachedStatus(){return status(lastConnected);}
    public String status(){
        return status(connected());
    }
    private String status(boolean connected){
        if(!connected)return "Limelight unavailable: shooting disabled; timed parking still active";
        if(!problem.isEmpty())return problem;
        if(teamTwo)return calibrated()?"Live own-Hive far Cell tags required before and during feeding":"TEAM TWO HIVE HEIGHT NOT CALIBRATED: shooting disabled";
        return calibrated()?"Live near-Cell tags required for Team One shots / yellow pollen pickup":"TEAM ONE HIVE HEIGHT NOT CALIBRATED: shooting disabled";
    }
    public Vision read(double now){
        Vision empty=Vision.empty(pipeline,now);
        if(!connected()||now-changedAt<.20)return empty;
        try{
            LLResult result=camera.getLatestResult();
            int expected=pipeline==0?Constants.LIMELIGHT_APRILTAG_PIPELINE:Constants.LIMELIGHT_COLOR_PIPELINE;
            if(result==null||!result.isValid()||result.getPipelineIndex()!=expected)return empty;
            double age=(result.getStaleness()+result.getCaptureLatency()+result.getTargetingLatency())/1000;
            double stamp=result.getTimestamp();
            // Match RobotIO.Vision freshness: this accepts a live frame that
            // arrived late, but never invents a detection from an old result.
            if(!Double.isFinite(age)||age<0||age>1.0||!Double.isFinite(stamp))return empty;
            if(!Double.isNaN(frameTimestamp)&&stamp<frameTimestamp)return empty;
            if(stamp!=frameTimestamp){frameTimestamp=stamp;capturedAt=now-age;}
            List<Target> targets=new ArrayList<Target>();
            if(pipeline==0){
                List<LLResultTypes.FiducialResult> tags=result.getFiducialResults();
                if(tags!=null)for(LLResultTypes.FiducialResult tag:tags)targets.add(new Target(tag.getFiducialId(),tag.getTargetXDegrees(),tag.getTargetYDegrees(),height(tag)));
            }else{
                // ColorResult has no color label: this pipeline MUST detect yellow pollen only.
                LLResultTypes.ColorResult target=largestColorTarget(result);
                if(target!=null)targets.add(new Target(-1,target.getTargetXDegrees(),target.getTargetYDegrees(),Double.NaN,target.getTargetArea(),"yellow"));
            }
            problem="";
            return new Vision(!targets.isEmpty(),pipeline,capturedAt,targets.toArray(new Target[0]));
        }catch(java.util.concurrent.CancellationException e){throw e;}
        catch(RuntimeException e){problem="Camera read failed: "+e.getMessage();return empty;}
    }
    /** Biggest color blob in view, or null when none. */
    private static LLResultTypes.ColorResult largestColorTarget(LLResult result){
        List<LLResultTypes.ColorResult> blobs=result.getColorResults();
        LLResultTypes.ColorResult largest=null;
        if(blobs!=null)for(LLResultTypes.ColorResult blob:blobs)if(largest==null||blob.getTargetArea()>largest.getTargetArea())largest=blob;
        return largest;
    }
    private double height(LLResultTypes.FiducialResult tag){
        if(!calibrated()||tag.getTargetPoseRobotSpace()==null)return Double.NaN;
        Position p=tag.getTargetPoseRobotSpace().getPosition();
        if(p==null)return Double.NaN;
        p=p.toUnit(DistanceUnit.INCH);
        int axis=teamTwo?Constants.TEAM_TWO_TAG_HEIGHT_AXIS:Constants.TEAM_ONE_TAG_HEIGHT_AXIS;
        double origin=teamTwo?Constants.TEAM_TWO_ROBOT_ORIGIN_HEIGHT_IN:Constants.TEAM_ONE_ROBOT_ORIGIN_HEIGHT_IN;
        return (axis==0?p.x:axis==1?p.y:p.z)+origin;
    }
    private boolean calibrated(){return teamTwo?Constants.TEAM_TWO_HIVE_HEIGHT_CALIBRATED:Constants.TEAM_ONE_HIVE_HEIGHT_CALIBRATED;}
    public void close(){if(camera!=null)camera.stop();}
}
