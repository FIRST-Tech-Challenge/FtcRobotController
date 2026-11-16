package org.firstinspires.ftc.teamcode.biobuzz.ftc;

import java.util.ArrayList;
import java.util.List;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO;

/** FTC SDK adapter. No socket, desktop Java or browser dependency is deployed to the robot. */
public final class FtcRobotIO implements RobotIO,AutoCloseable {
    private final FtcHardwareConfig config;
    private final DcMotor fl,fr,bl,br,intake,shooter;
    private final CRServo feeder;
    private final GoBildaPinpointDriver pinpoint;
    private final Limelight3A limelight;
    private final Servo cameraTilt;
    private final FtcHardwareConfig.BallCounter counter;
    private final long epoch=System.nanoTime();
    private final org.firstinspires.ftc.teamcode.biobuzz.FeedCadence cadence=new org.firstinspires.ftc.teamcode.biobuzz.FeedCadence();
    private int pipeline=-1;
    private double frameTimestamp=Double.NaN,frameCapturedAt,pipelineChangedAt;
    private final java.util.function.BooleanSupplier stopRequested;
    private Pose lastPose;
    private boolean actuatorsStopped,closed;
    public FtcRobotIO(HardwareMap map,FtcHardwareConfig config,int alliance) {
        this(map,config,alliance,()->Thread.currentThread().isInterrupted());
    }
    public FtcRobotIO(HardwareMap map,FtcHardwareConfig config,int alliance,java.util.function.BooleanSupplier stopRequested) {
        this.stopRequested=stopRequested;checkStop();lastPose=config.start[alliance];
        config.validate();this.config=config;counter=config.createBallCounter(map);
        fl=map.get(DcMotor.class,config.frontLeft);fr=map.get(DcMotor.class,config.frontRight);
        bl=map.get(DcMotor.class,config.backLeft);br=map.get(DcMotor.class,config.backRight);
        intake=map.get(DcMotor.class,config.intake);shooter=map.get(DcMotor.class,config.shooter);
        feeder=map.get(CRServo.class,config.feeder);
        pinpoint=map.get(GoBildaPinpointDriver.class,config.pinpoint);
        checkStop();
        limelight=config.useVision?map.get(Limelight3A.class,config.limelight):null;
        cameraTilt=!config.useVision||config.cameraTilt==null||config.cameraTilt.trim().isEmpty()?null:map.tryGet(Servo.class,config.cameraTilt);
        fl.setDirection(DcMotor.Direction.REVERSE);bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);br.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);shooter.setDirection(DcMotor.Direction.FORWARD);
        for(DcMotor motor:new DcMotor[]{fl,fr,bl,br,intake})motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        for(DcMotor motor:new DcMotor[]{fl,fr,bl,br,intake,shooter})motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setDirection(CRServo.Direction.REVERSE);
        apply(new Outputs());
        checkStop();
        pinpoint.setOffsets(config.podXOffsetMm,config.podYOffsetMm,DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(config.reverseXPod?GoBildaPinpointDriver.EncoderDirection.REVERSED:GoBildaPinpointDriver.EncoderDirection.FORWARD,
            config.reverseYPod?GoBildaPinpointDriver.EncoderDirection.REVERSED:GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.recalibrateIMU();Pose p=config.start[alliance];pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,p.x,p.y,AngleUnit.RADIANS,p.heading));
        checkStop();
        if(limelight!=null)limelight.start();
    }
    public double nowSeconds() { return (System.nanoTime()-epoch)/1e9; }
    public boolean stopRequested(){return stopRequested.getAsBoolean()||Thread.currentThread().isInterrupted();}
    private void checkStop(){if(stopRequested())throw new java.util.concurrent.CancellationException("FTC operation cancelled by STOP");}
    public Sensors readSensors() {
        checkStop();pinpoint.update();checkStop();Pose2D p=SafePinpointPose.read(pinpoint);double now=nowSeconds();
        boolean valid=p!=null;
        if(valid)lastPose=new Pose(p.getX(DistanceUnit.INCH),p.getY(DistanceUnit.INCH),p.getHeading(AngleUnit.RADIANS));
        Pose pose=lastPose; // Not marked valid when calibration/read failed.
        LLResult result=limelight==null?null:limelight.getLatestResult();Vision vision=Vision.empty(pipeline,now);
        int actual=pipeline==0?config.tagPipeline:config.ballPipeline;
        if(result!=null&&result.isValid()&&result.getPipelineIndex()==actual&&now-pipelineChangedAt>=.20){
            List<Target> targets=new ArrayList<Target>();
            if(pipeline==0){List<LLResultTypes.FiducialResult> tags=result.getFiducialResults();if(tags!=null)for(LLResultTypes.FiducialResult t:tags)
                targets.add(new Target(t.getFiducialId(),t.getTargetXDegrees(),t.getTargetYDegrees(),tagHeight(t)));}
            else targets.add(new Target(-1,result.getTx(),result.getTy()));
            double age=Math.max(0,result.getStaleness()+result.getCaptureLatency()+result.getTargetingLatency())/1000;
            if(result.getTimestamp()!=frameTimestamp){frameTimestamp=result.getTimestamp();frameCapturedAt=now-age;}
            vision=new Vision(!targets.isEmpty()&&age<=.25,pipeline,frameCapturedAt,targets.toArray(new Target[0]));
        }
        return new Sensors(pose,now,valid,limelight!=null&&limelight.isConnected(),counter.count(),vision);
    }
    private double tagHeight(LLResultTypes.FiducialResult tag) {
        if(!config.hiveHeightCalibrated||tag.getTargetPoseRobotSpace()==null)return Double.NaN;
        Position p=tag.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
        return (config.tagHeightAxis==0?p.x:config.tagHeightAxis==1?p.y:p.z)+config.robotOriginHeightInches;
    }
    public void apply(Outputs o) {
        o=o.copy();if(config.movementOnly){o.intakePower=o.shooterPower=o.feederPower=0;o.pipeline=0;}
        if(stopRequested()||o.forward==0&&o.strafeRight==0&&o.turnClockwise==0&&o.intakePower==0&&o.shooterPower==0&&o.feederPower==0){cadence.apply(0,nowSeconds());stopActuators();return;}
        actuatorsStopped=false;
        double[] w=o.wheelPowers();
        fl.setPower(w[0]);fr.setPower(w[1]);bl.setPower(w[2]);br.setPower(w[3]);
        intake.setPower(o.intakePower);
        shooter.setPower(o.shooterPower);feeder.setPower(cadence.apply(o.shooterPower>0?o.feederPower:0,nowSeconds()));
        if(pipeline!=o.pipeline){
            pipeline=o.pipeline;pipelineChangedAt=nowSeconds();frameTimestamp=Double.NaN;
            if(cameraTilt!=null)cameraTilt.setPosition(pipeline==0?config.tiltUp:config.tiltDown);
            if(limelight!=null)limelight.pipelineSwitch(pipeline==0?config.tagPipeline:config.ballPipeline);
        }
    }
    private void stopActuators(){
        if(actuatorsStopped)return;actuatorsStopped=true;
        SafeShutdown.run(()->fl.setPower(0),()->fr.setPower(0),()->bl.setPower(0),()->br.setPower(0),
            ()->intake.setPower(0),()->shooter.setPower(0),()->feeder.setPower(0));
    }
    public void close() { if(closed)return;closed=true;stopActuators();if(limelight!=null)SafeShutdown.run(limelight::stop); }
}
