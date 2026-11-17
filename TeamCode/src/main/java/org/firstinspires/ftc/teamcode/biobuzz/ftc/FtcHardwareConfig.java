package org.firstinspires.ftc.teamcode.biobuzz.ftc;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.biobuzz.FieldLayout;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.Pose;

/** Hardware calibration only; the autonomous algorithm lives in the parent biobuzz package. */
public class FtcHardwareConfig {
    public boolean calibrated=false;
    /** Default to fixed-pose autonomy; explicitly enable only with calibrated vision. */
    public boolean useVision=false;
    public boolean movementOnly=true;
    public final FieldLayout field=new FieldLayout();
    public final Pose[] start={field.hive[0],field.hive[1]};
    public String frontLeft="left_front_drive",frontRight="right_front_drive",backLeft="left_back_drive",backRight="right_back_drive";
    public String intake="intake",shooter="shooter",feeder="shooter_servo";
    public String pinpoint="odometry_1",limelight="limelight";
    /** Optional positional tilt servo; missing, null or empty means fixed camera mounting. */
    public String cameraTilt="limelight_tilt_servo";
    public double podXOffsetMm=0,podYOffsetMm=0,tiltUp=.3,tiltDown=.7;
    public boolean reverseXPod=false,reverseYPod=false;
    public int tagPipeline=1,ballPipeline=0;
    /** Confirm Limelight camera pose/extrinsics at tiltUp before using 3D heights. */
    public boolean hiveHeightCalibrated=false;
    /** Robot-space vertical axis: 2 for Z-up. Set only after verifying the pose convention. */
    public int tagHeightAxis=2;
    public double robotOriginHeightInches=0;
    public double shooterPower=.395;
    /** Configure four occupied-slot signals, or override createBallCounter with your actual sensing mechanism. */
    public String[] ballSlotNames={};
    public boolean occupiedSignalLow=true;
    public interface BallCounter { int count(); }
    public BallCounter createBallCounter(HardwareMap map) {
        if(movementOnly)return ()->-1;
        if(!useVision&&ballSlotNames.length==0)return ()->-1; // Unknown, not a fabricated count.
        if(ballSlotNames.length!=4)throw new IllegalStateException("Configure ballSlotNames or provide a sensor-backed BallCounter; the old SoftwareIndexer assumes counts.");
        final DigitalChannel[] slots=new DigitalChannel[4];
        for(int i=0;i<4;i++){slots[i]=map.get(DigitalChannel.class,ballSlotNames[i]);slots[i].setMode(DigitalChannel.Mode.INPUT);}
        return ()->{int count=0;for(DigitalChannel slot:slots)if(slot.getState()!=occupiedSignalLow)count++;return count;};
    }
    public void validate() {
        if(movementOnly&&useVision)throw new IllegalArgumentException("Movement-only requires useVision=false");
        if(!calibrated)throw new IllegalStateException("Calibrate field/start poses, pod offsets/directions and shooter power in FtcHardwareConfig; configure vision/count sensors if used, then mark calibrated=true.");
        if(useVision&&(!hiveHeightCalibrated||tagHeightAxis<0||tagHeightAxis>2||!Double.isFinite(robotOriginHeightInches)))
            throw new IllegalStateException("Calibrate robot-space tag heights at tiltUp, including camera pose, vertical axis and origin height.");
        if(shooterPower<0||shooterPower>1||!Double.isFinite(shooterPower))throw new IllegalArgumentException("Shooter power");
        for(Pose[] poses:new Pose[][]{start,field.hive,field.farHive,field.garden,field.flower})for(Pose p:poses)
            if(p==null||!p.finite()||!field.allowed(p))throw new IllegalArgumentException("Invalid field calibration pose");
        for(int[][] groups:new int[][][]{field.nearTags,field.farTags})for(int[] ids:groups)
            if(ids.length==0)throw new IllegalArgumentException("Missing hive AprilTag IDs");
    }
}
