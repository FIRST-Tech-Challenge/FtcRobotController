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
import org.firstinspires.ftc.teamcode.quantumstorm.Auto.LimeServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.biobuzz.ShootingAim;

/** Actual SDK observations; never invents detections, tilt state or inventory. */
public final class TeamOneLimelight implements AutoCloseable {
    private final Limelight3A camera;
    private final Servo tilt;
    private final LimeServo servo;
    private final DoubleSupplier clock;
    private final boolean teamTwo;
    private final String hardwareName;
    private boolean lastConnected;
    private int pipeline=-1;
    private boolean idleScan;
    private boolean pickupScan;
    private double changedAt,frameTimestamp=Double.NaN,capturedAt;
    private String problem="";
    private LLResult lastResult;
    private Vision lastVision=Vision.empty(0,0);
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
        servo=new LimeServo(camera,tilt,clock);
        if(camera!=null){try{camera.start();reset();}catch(RuntimeException e){problem=e.getMessage();}}
    }
    public void reset(){pipeline=-1;idleScan=false;select(0);}
    public void select(int mode){
        select(mode,false);
    }
    public void select(int mode,boolean scanWhileIdle){
        select(mode,scanWhileIdle,false);
    }
    public void select(int mode,boolean scanWhileIdle,boolean scanWhilePickingUp){
        if(Thread.currentThread().isInterrupted())throw new java.util.concurrent.CancellationException("STOP");
        if(scanWhilePickingUp){
            idleScan=false;pickupScan=true;pipeline=1;
            int previous=servo.getPipelineIndex();
            servo.updatePickup();
            if(previous!=servo.getPipelineIndex()){changedAt=clock.getAsDouble();frameTimestamp=Double.NaN;}
            return;
        }
        if(scanWhileIdle){
            pickupScan=false;idleScan=true;pipeline=mode;
            int previous=servo.getPipelineIndex();
            servo.update(false);
            if(previous!=servo.getPipelineIndex()){changedAt=clock.getAsDouble();frameTimestamp=Double.NaN;}
            return;
        }
        if(mode==pipeline&&!idleScan&&!pickupScan)return;
        idleScan=false;pickupScan=false;
        pipeline=mode;changedAt=clock.getAsDouble();frameTimestamp=Double.NaN;
        try{
            servo.selectPipeline(mode==0?LimeServo.Pipeline.APRIL_TAG:LimeServo.Pipeline.YELLOW);
        }catch(RuntimeException e){problem=e.getMessage();}
    }
    public boolean mapped(){return camera!=null;}
    public String hardwareName(){return hardwareName;}
    public int selectedPipeline(){return servo.getPipelineIndex();}
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
        lastResult=null;lastVision=empty;
        if(!connected())return empty;
        try{
            LLResult result=servo.getLatestResult();
            lastResult=result;
            int expected=selectedPipeline();
            if(now-changedAt<.20||!servo.isFreshResult(result)||result.getPipelineIndex()!=expected)return empty;
            double age=(result.getStaleness()+result.getCaptureLatency()+result.getTargetingLatency())/1000;
            double stamp=result.getTimestamp();
            // Match RobotIO.Vision freshness: this accepts a live frame that
            // arrived late, but never invents a detection from an old result.
            if(!Double.isFinite(age)||age<0||age>1.0||!Double.isFinite(stamp))return empty;
            if(!Double.isNaN(frameTimestamp)&&stamp<frameTimestamp)return empty;
            if(stamp!=frameTimestamp){frameTimestamp=stamp;capturedAt=now-age;}
            List<Target> targets=new ArrayList<Target>();
            if(pipeline==0&&!idleScan){
                List<LLResultTypes.FiducialResult> tags=result.getFiducialResults();
                if(tags!=null)for(LLResultTypes.FiducialResult tag:tags)targets.add(new Target(tag.getFiducialId(),tag.getTargetXDegrees(),tag.getTargetYDegrees(),height(tag)));
            }else{
                // SDK blobs have no label: the configured pipeline defines the color.
                LLResultTypes.ColorResult target=largestColorTarget(result);
                String color=servo.getActivePipeline()==LimeServo.Pipeline.RED?"red"
                        :servo.getActivePipeline()==LimeServo.Pipeline.BLUE?"blue":"yellow";
                if(target!=null)targets.add(new Target(-1,target.getTargetXDegrees(),target.getTargetYDegrees(),Double.NaN,target.getTargetArea(),color));
            }
            problem="";
            lastVision=new Vision(result.isValid()&&!targets.isEmpty(),idleScan?2:pipeline,capturedAt,targets.toArray(new Target[0]),result.getTx());
            return lastVision;
        }catch(java.util.concurrent.CancellationException e){throw e;}
        catch(RuntimeException e){problem="Camera read failed: "+e.getMessage();return empty;}
    }

    /** Report the already sampled frame; telemetry must not resample camera hardware. */
    public void addTelemetry(Telemetry telemetry,double now,boolean shootingAuthorized,Vision aimingVision){
        boolean targetValid=lastConnected&&lastResult!=null
                &&lastResult.getPipelineIndex()==selectedPipeline()
                &&aimingVision.fresh(now,aimingVision.pipeline)&&now-aimingVision.capturedAt<=.25;
        telemetry.addData("tv",lastResult!=null&&lastResult.isValid()?"1":"0");
        telemetry.addData("tx",!Double.isFinite(aimingVision.tx)?"N/A":String.format(java.util.Locale.US,"%.2f deg",aimingVision.tx));
        telemetry.addData("ty",lastResult==null?"N/A":String.format(java.util.Locale.US,"%.2f deg",lastResult.getTy()));
        telemetry.addData("Shooting authorized",shootingAuthorized?"YES":"NO");
        List<LLResultTypes.FiducialResult> tags=lastResult==null?null:lastResult.getFiducialResults();
        List<LLResultTypes.ColorResult> colors=lastResult==null?null:lastResult.getColorResults();
        telemetry.addData("AprilTags seen",tags==null?0:tags.size());
        telemetry.addData("Colors seen",colors==null?0:colors.size());
        telemetry.addData("Turn suggestion",ShootingAim.turnSuggestion(targetValid,aimingVision.tx));
        double angle=servo.getCommandedTiltDegrees();
        telemetry.addData("Camera servo",Double.isFinite(angle)&&Math.abs(angle)<=1.0?"NEUTRAL":"NOT NEUTRAL");
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
