package org.firstinspires.ftc.teamcode.biobuzz;

import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** QuantumStorm's autonomous sequence, independent of FTC SDK, browser, or wall-clock timers. */
public final class AutoStateMachine implements RobotController {
    public static final double PARK_START_SECONDS=26.0;
    public static final double AUTO_END_SECONDS=30.0;
    public enum State { INIT, MOVING, SHOOTING, INTAKE, PRE_PARK, EXTRA_PICKUP, SAFETY_HOLD, ERROR, WAIT_TILT, WAITING, PARK, STOPPED }
    public enum Routine { FULL, TEAM_ONE, TEAM_TWO }
    private enum ShootStep { CHECK_BALLS, AIM, FIRE }
    private enum Source { GARDEN, FLOWER, NEAR_FLOWER }
    private final RobotIO io;
    private final FieldLayout field;
    private final Navigator navigator;
    private final boolean useVision;
    private final Routine routine;
    private final boolean movementOnly;
    private final boolean collectAfterRoutine;
    /** Nominal measured-inventory simulator dwell times; not inferred ball counts. */
    public static final double DRY_GARDEN_SECONDS=.14,DRY_FLOWER_SECONDS=2.34;
    /** Timed attempts when inventory is unavailable; never manufacture a ball count. */
    public double timedFeedSeconds=3.0,timedPickupSeconds=3.0;
    /** Position-only parking acceptance; hardware may allow more odometry noise. */
    public double parkingToleranceInches=.65;
    /** Finish scoring early, but hold all outputs OFF until the 26-second park trigger. */
    public boolean waitUntilParkTime=false;
    private boolean parkingComplete;
    private HiveVision hiveVision;
    private State state=State.STOPPED, afterMove;
    private ShootStep shootStep;
    private Source source;
    private Pose destination;
    private Pose currentPose;
    private String destinationName="",error="";
    private boolean running,gardenDone,flowerDone,flipped,recoverable;
    private boolean flowerTiltConfirmed;
    private boolean extraAttempted,routeFailed;
    private Pose extraSweep;
    private State resumeState;
    private double holdAt,resumeStateAt;
    private boolean fallbackActive;
    private int finalShotHiveState;
    private int alliance,retries;
    private int shotBatches;
    private boolean preParkPickup;
    private double preParkPickupEnds;
    private double startAt,stateAt,lastNow,power;
    private double shooterAt,moveDeadline;
    private Pose progressPose;
    private double progressAt;
    private Outputs last=new Outputs();
    public AutoStateMachine(RobotIO io,FieldLayout field) { this(io,field,true); }
    public AutoStateMachine(RobotIO io,FieldLayout field,boolean useVision) {
        this(io,field,useVision,Routine.FULL);
    }
    public AutoStateMachine(RobotIO io,FieldLayout field,boolean useVision,Routine routine) {
        this(io,field,useVision,routine,false);
    }
    public AutoStateMachine(RobotIO io,FieldLayout field,boolean useVision,Routine routine,boolean movementOnly) {
        this(io,field,useVision,routine,movementOnly,routine!=Routine.TEAM_ONE);
    }
    /** Hardware can disable the final optional collection without changing required pickups. */
    public AutoStateMachine(RobotIO io,FieldLayout field,boolean useVision,Routine routine,boolean movementOnly,boolean collectAfterRoutine) {
        if(routine==null)throw new IllegalArgumentException("Routine is required");
        if(movementOnly&&useVision)throw new IllegalArgumentException("Movement-only uses fixed simulator poses, not vision");
        if(useVision&&(routine==Routine.TEAM_ONE||routine==Routine.TEAM_TWO))throw new IllegalArgumentException("Partner roles require camera-free mode");
        this.movementOnly=movementOnly;
        this.collectAfterRoutine=collectAfterRoutine;
        this.routine=routine;
        this.io=io;this.field=field;this.useVision=useVision;this.navigator=new Navigator(field,io::stopRequested);
    }
    public void start(int alliance,double power) {
        if(alliance<0||alliance>1||!Double.isFinite(power)||power<0||power>1)throw new IllegalArgumentException("Invalid alliance or shooter power");
        if(!Double.isFinite(parkingToleranceInches)||parkingToleranceInches<=0||parkingToleranceInches>3)throw new IllegalArgumentException("Parking tolerance must be greater than zero and at most 3 inches");
        this.alliance=alliance;this.power=power;startAt=lastNow=io.nowSeconds();running=true;retries=0;
        hiveVision=useVision?new HiveVision(field,alliance):null;
        navigator.useScriptedRoute(!useVision,alliance);
        gardenDone=flowerDone=flipped=flowerTiltConfirmed=extraAttempted=routeFailed=false;finalShotHiveState=-1;error="";destination=null;enter(State.INIT);io.apply(new Outputs());
        shotBatches=0;flipped=routine==Routine.TEAM_TWO;
        preParkPickup=false;
        parkingComplete=false;
        resumeState=null;fallbackActive=false;currentPose=null;
        shooterAt=Double.NaN;
    }
    public void stop() { boolean alreadyStopped=!running&&state==State.STOPPED;running=false;state=State.STOPPED;last=new Outputs();if(!alreadyStopped)io.apply(last); }
    public boolean isRunning() { return running; }
    public State getState() { return state; }
    public String lastError() { return error; }
    public Outputs getOutputs() { return last.copy(); }
    public Pose destination() { return running&&(state==State.MOVING||state==State.PARK)?destination:null; }
    public java.util.List<Pose> route() { return running&&(state==State.MOVING||state==State.PARK)?navigator.route():new java.util.ArrayList<Pose>(); }
    public double elapsed() { return Math.max(0,io.nowSeconds()-startAt); }
    public String status() {
        return state+(state==State.MOVING?" → "+destinationName:state==State.SHOOTING?" · "+shootStep:state==State.INTAKE?" · "+source:"")
            +String.format(java.util.Locale.ROOT," · %.1f / 30 s",elapsed())
            +(waitUntilParkTime?" · wait after routine; park starts at 26 s":routine==Routine.TEAM_ONE||routine==Routine.TEAM_TWO?(collectAfterRoutine?" · optional pollen pickup, then park (26 s fallback)":" · park after routine; no extra pickup"):" · park starts at 26 s")
            +(routine==Routine.TEAM_ONE?" · Team 1: four preloads":routine==Routine.TEAM_TWO?" · Team 2: partner first tilt assumed; preloads, Garden":"")
            +(extraAttempted?" · Extra pollen sweep attempted":"")
            +(parkingComplete?" · Parked: all outputs OFF":"")
            +(movementOnly?" · MOVEMENT ONLY: mechanisms OFF, simulator dwell timing":"")
            +(!useVision?" · Camera-free: fixed route, no tag detection, Hive tilt unverified":"")
            +(navigator.isPlanning()?" · Planning route (STOP available)":"")
            +(flowerTiltConfirmed?" · Flower tilt confirmed":"")+(error.isEmpty()?"":" · "+error);
    }
    private void enter(State state) { this.state=state;stateAt=io.nowSeconds();navigator.reset();if(state==State.SHOOTING)shootStep=ShootStep.CHECK_BALLS; }
    private void moveThen(Pose target,String name,State after) {
        destination=target;destinationName=name;afterMove=after;enter(State.MOVING);
        if(movementOnly){fallbackActive=false;navigator.useScriptedRoute(true,alliance);}
        Pose current=currentPose;
        progressPose=current;progressAt=io.nowSeconds();
        // Never run a synchronous search just to estimate a deadline.
        double distance=current.distance(target)*1.6;
        moveDeadline=Math.max(3,distance/Math.max(1,25*field.drivePower)+5);
    }
    private Pose hive() { return flipped?field.farHive[alliance]:field.hive[alliance]; }
    private void pickup() {
        if(!gardenDone){source=Source.GARDEN;moveThen(field.garden[alliance],"Garden",State.INTAKE);}
        else if(!flowerDone){source=Source.FLOWER;moveThen(field.flower[alliance],"Flower",State.INTAKE);}
        else{finalShotHiveState=flipped?1:0;enter(useVision?State.WAIT_TILT:State.WAITING);}
    }
    private void beginParking(){preParkPickup=false;destination=field.parkingTarget(currentPose,alliance);enter(State.PARK);}
    private void parkAfterRoutine(){
        if(waitUntilParkTime&&elapsed()<PARK_START_SECONDS){destination=null;enter(State.WAITING);}
        else beginParking();
    }
    private void finishShot(){
        shooterAt=Double.NaN;shotBatches++;
        if(routine==Routine.TEAM_ONE){
            parkAfterRoutine();return;
        }
        if(routine==Routine.TEAM_TWO&&shotBatches>=2){enter(State.WAIT_TILT);return;}
        pickup();
    }
    private void finishPickup() {
        if(source==Source.GARDEN)gardenDone=true;if(source==Source.FLOWER)flowerDone=true;
        if(preParkPickup){preParkPickup=false;parkAfterRoutine();return;}
        if(!useVision){
            // Scripted far-side return, matching the default simulation sequence.
            // This is an assumption, not a detected/confirmed Hive tip.
            flipped=true;moveThen(field.farHive[alliance],"Hive scripted far face",State.SHOOTING);return;
        }
        // Move to the closer observation side; vision, never proximity, decides
        // whether that Cell is up and authorizes feeding after arrival.
        Pose observation=currentPose.distance(field.hive[alliance])<=currentPose.distance(field.farHive[alliance])
            ?field.hive[alliance]:field.farHive[alliance];
        moveThen(observation,"Hive",State.SHOOTING);
    }
    private void fail(String reason,boolean recoverable) { routeFailed=true;error=reason;this.recoverable=recoverable;enter(State.ERROR); }
    private void safetyHold(String reason,double now){
        error=reason;
        if(state!=State.SAFETY_HOLD&&state!=State.PARK){resumeState=state;resumeStateAt=stateAt;holdAt=now;enter(State.SAFETY_HOLD);}
    }
    /** One bounded search of the known pollen outlet, not camera-dependent ball detection. */
    private void tryExtraPickup(int count) {
        if(waitUntilParkTime)return;
        if(!collectAfterRoutine)return;
        if(routine!=Routine.FULL||extraAttempted||routeFailed||!gardenDone||!flowerDone||!movementOnly&&count>=4)return;
        Pose flower=field.flower[alliance];
        // Inset from the wall so the forward sweep clears the Flower's housing.
        Pose start=new Pose(flower.x+(alliance==0?2:-2),flower.y,flower.heading);
        Pose sweep=new Pose(start.x+8*Math.cos(start.heading),start.y+8*Math.sin(start.heading),start.heading);
        double speed=Math.max(1,20*field.drivePower);
        // Budget rotation, approach, sweep and margin before the 26-second cutoff.
        double needed=currentPose.distance(start)*1.6/speed+2.4;
        double parkNeeded=Math.max(start.distance(field.parkingTarget(start,alliance)),sweep.distance(field.parkingTarget(sweep,alliance)))/speed+.6;
        if(elapsed()+needed>25||parkNeeded>4||!field.canRotate(currentPose,start.heading)
                ||!field.clear(currentPose,start,start.heading)||!field.clear(start,sweep,start.heading)
                ||!field.clear(start,field.parkingTarget(start,alliance),start.heading)
                ||!field.clear(sweep,field.parkingTarget(sweep,alliance),start.heading))return;
        extraAttempted=true;extraSweep=sweep;moveThen(start,"Flower leftover pollen",State.EXTRA_PICKUP);
    }
    private Target tag(Vision vision,double now) {
        return hiveVision.aim(now,flipped);
    }
    /** One bounded optional Flower attempt; retain the pollen and reserve parking time. */
    private void collectBeforePark(int count){
        if(!collectAfterRoutine){parkAfterRoutine();return;}
        if(extraAttempted||routeFailed||!movementOnly&&count>=4){beginParking();return;}
        Pose target=currentPose.distance(field.nearFlower[alliance])<currentPose.distance(field.flower[alliance])
            ?field.nearFlower[alliance]:field.flower[alliance];
        double speed=Math.max(1,25*field.drivePower);
        double approach=currentPose.distance(target)*1.6/Math.max(1,20*field.drivePower)+1.2;
        double park=target.distance(field.parkingTarget(target,alliance))*1.15/speed+.8;
        if(!field.allowed(target)||park>4||elapsed()+approach+3>25||elapsed()+approach+3+park+.5>=30){beginParking();return;}
        preParkPickup=true;extraAttempted=true;preParkPickupEnds=Math.min(25,AUTO_END_SECONDS-park-.5);
        source=target==field.nearFlower[alliance]?Source.NEAR_FLOWER:Source.FLOWER;
        moveThen(target,"Extra Flower pollen",State.INTAKE);
    }
    public void tick() {
        if(io.stopRequested()){stop();return;}
        Outputs out=new Outputs();double now=io.nowSeconds();
        if(!running){last=out;io.apply(out);return;}
        Sensors sensors;String readError="";
        try{sensors=io.readSensors();}
        catch(java.util.concurrent.CancellationException e){stop();return;}
        catch(RuntimeException e){
            if(!movementOnly)throw e;
            readError="Sensor read failed: "+e.getMessage();
            sensors=new Sensors(currentPose==null?field.hive[alliance]:currentPose,io.nowSeconds(),false,false,-1,Vision.empty(0,io.nowSeconds()));
        }
        now=io.nowSeconds();
        if(io.stopRequested()){stop();return;}
        if(!Double.isFinite(now)||now<lastNow){fail("Clock moved backward",false);running=false;last=out;io.apply(out);return;}lastNow=now;
        if(elapsed()>=AUTO_END_SECONDS){running=false;error="30-second autonomous limit; all actuators stopped";last=out;io.apply(out);return;}
        // A safety hold keeps the loop alive but never drives on invalid measurements.
        if(movementOnly&&state!=State.PARK&&elapsed()>=PARK_START_SECONDS){
            destination=currentPose==null?null:field.parkingTarget(currentPose,alliance);resumeState=null;enter(State.PARK);
        }
        if(!sensors.odometryValid||!sensors.pose.finite()||now-sensors.sampledAt<0||now-sensors.sampledAt>.25){
            if(movementOnly){safetyHold(readError.isEmpty()?"Odometry unavailable or stale; drive held OFF":readError,now);last=out;io.apply(out);return;}
            fail("Odometry unavailable or stale",false);running=false;last=out;io.apply(out);return;
        }
        if(!movementOnly&&(sensors.ballCount<-1||sensors.ballCount>4||(useVision&&(sensors.ballCount<0||!sensors.cameraConnected)))){
            fail(sensors.ballCount<0||sensors.ballCount>4?"Ball-count sensor unavailable":"Camera disconnected",false);running=false;last=out;io.apply(out);return;
        }
        if(useVision)hiveVision.update(sensors.vision,now);
        currentPose=sensors.pose;
        if(movementOnly&&state==State.SAFETY_HOLD){
            double held=now-holdAt;state=resumeState;resumeState=null;stateAt=resumeStateAt+held;
            if(!Double.isNaN(shooterAt))shooterAt+=held;
            navigator.reset();progressPose=currentPose;progressAt=now;error="";
        }
        if(state==State.PARK&&destination==null)destination=field.parkingTarget(currentPose,alliance);
        // The final four seconds belong exclusively to parking, regardless of
        // which scoring or pickup state was active before the transition.
        if(state!=State.PARK&&elapsed()>=PARK_START_SECONDS){
            beginParking();
        }
        else if(preParkPickup&&(elapsed()>=preParkPickupEnds||!movementOnly&&sensors.ballCount>=4)){
            preParkPickup=false;beginParking();
        }
        else if(extraAttempted&&(state==State.EXTRA_PICKUP||state==State.MOVING&&afterMove==State.EXTRA_PICKUP)
                &&(elapsed()>=25||!movementOnly&&sensors.ballCount>=4))enter(State.WAITING);
        double age=now-stateAt;
        State before=state;
        try {
            switch(state) {
                case INIT:moveThen(hive(),"Hive",State.SHOOTING);break;
                case PRE_PARK:collectBeforePark(sensors.ballCount);break;
                case MOVING:
                    if(afterMove==State.INTAKE&&source==Source.NEAR_FLOWER&&sensors.pose.distance(destination)<.65
                            &&Math.abs(Pose.angle(destination.heading-sensors.pose.heading))<Math.toRadians(2)){
                        enter(State.INTAKE);break;
                    }
                    // Pickup needs a usable approach, not sub-inch shooting
                    // accuracy. Stop before intaking; never loosen shot alignment.
                    if(!useVision&&afterMove==State.INTAKE&&source==Source.GARDEN&&sensors.pose.distance(destination)<=3
                            &&Math.abs(Pose.angle(destination.heading-sensors.pose.heading))<=Math.toRadians(8)){
                        enter(State.INTAKE);break;
                    }
                    Outputs drive=navigator.move(sensors.pose,destination);
                    if(drive==null){if(afterMove==State.PARK)beginParking();else enter(afterMove);}else out=drive;
                    if(state==State.MOVING){
                        if(navigator.isPlanning()){
                            progressPose=sensors.pose;progressAt=now;stateAt=now;
                        }else if(!useVision){
                            // Real wheel speeds need not match the simulator's
                            // 25 in/s. Keep navigating while odometry shows progress,
                            // but stop a genuinely stalled robot after three seconds.
                            if(sensors.pose.distance(progressPose)>.25||Math.abs(Pose.angle(sensors.pose.heading-progressPose.heading))>Math.toRadians(3)){
                                progressPose=sensors.pose;progressAt=now;
                            }
                            if(now-progressAt>3)fail("Movement stalled toward "+destinationName,false);
                        }else if(age>moveDeadline)fail("Destination timeout: "+destinationName,false);
                    }
                    break;
                case SHOOTING:
                    if(!useVision){
                        // Fixed routine: always execute this bounded shot attempt,
                        // even with zero/unknown balls. Never infer shots or inventory.
                        out.shooterPower=power;
                        if(Double.isNaN(shooterAt))shooterAt=now;
                        if(shootStep==ShootStep.CHECK_BALLS){shootStep=ShootStep.AIM;stateAt=now;age=0;}
                        if(shootStep==ShootStep.AIM&&now-shooterAt>=1){shootStep=ShootStep.FIRE;stateAt=now;age=0;}
                        if(shootStep==ShootStep.FIRE){
                            if(age>=timedFeedSeconds){finishShot();break;}
                            out.feederPower=1;
                        }
                        break;
                    }
                    int observed=hiveVision.state(now);
                    if(observed>=0){
                        flipped=observed==1;
                        if(destination!=hive()){moveThen(hive(),flipped?"Hive far face":"Hive normal face",State.SHOOTING);break;}
                    }
                    if(shootStep==ShootStep.CHECK_BALLS){
                        if(sensors.ballCount==0){pickup();break;}shootStep=ShootStep.AIM;stateAt=now;age=0;
                    }
                    out.shooterPower=power;
                    if(Double.isNaN(shooterAt))shooterAt=now;
                    if(shootStep==ShootStep.AIM){
                        Target t=tag(sensors.vision,now);
                        if(t!=null&&Math.abs(t.tx)<2&&now-shooterAt>=1){shootStep=ShootStep.FIRE;stateAt=now;age=0;}
                        else if(age>3){fail(t==null?"AprilTag not found":"AprilTag alignment timeout",true);break;}
                        else if(t!=null)out.turnClockwise=FieldLayout.clip(t.tx*.02,.6);
                    }
                    if(shootStep==ShootStep.FIRE){
                        if(sensors.ballCount==0){out.shooterPower=0;shooterAt=Double.NaN;pickup();break;}
                        Target fireTarget=tag(sensors.vision,now);
                        if(age>3){fail(fireTarget==null?"Hive state or AprilTag lost while feeding":"Balls stuck in indexer",true);break;}
                        if(fireTarget!=null&&Math.abs(fireTarget.tx)<2)out.feederPower=1;
                    }
                    break;
                case INTAKE:
                    if(preParkPickup&&age>=3){finishPickup();break;}
                    if(movementOnly&&age+1e-9>=(source==Source.GARDEN?DRY_GARDEN_SECONDS:DRY_FLOWER_SECONDS)){finishPickup();break;}
                    if(!movementOnly&&sensors.ballCount>=4){finishPickup();break;}
                    // Garden intake already runs on approach; reserve the longer
                    // dwell for the Flower's release and sweep.
                    if(!movementOnly&&!useVision&&sensors.ballCount<0&&age>=(source==Source.GARDEN?Math.min(1,timedPickupSeconds):timedPickupSeconds)){finishPickup();break;}
                    if(!movementOnly&&!useVision&&age>=timedPickupSeconds){
                        // Preserve the measured count, including zero. A partial
                        // pickup does not cancel the remaining Garden/Flower route.
                        error="Pickup attempt ended with "+sensors.ballCount+" measured balls; continuing routine";
                        finishPickup();break;
                    }
                    if(age>4){fail("Pickup did not fill four slots",false);break;}
                    out.pipeline=1;out.intakePower=1;
                    // Touch at a corner, then strafe clear of the outlet. Allow
                    // staged releases to clear before sweeping them through the
                    // front intake; only the measured count completes pickup.
                    if(source!=Source.GARDEN&&age<.12)out.forward=.3;
                    else if(source!=Source.GARDEN&&age<.24)out.strafeRight=-.4;
                    else if(source!=Source.GARDEN&&age<1.24)out.strafeRight=.2;
                    else if(source!=Source.GARDEN&&age<2.24)out.forward=0;
                    else if(source!=Source.GARDEN)out.forward=.65;
                    else if(!useVision){
                        // Finish the short, collision-checked Garden approach
                        // while intaking, then leave on the bounded pickup timer.
                        Outputs approach=navigator.move(sensors.pose,destination);
                        if(approach!=null){out.forward=approach.forward;out.strafeRight=approach.strafeRight;out.turnClockwise=approach.turnClockwise;}
                    }
                    else if(sensors.vision.fresh(now,1)&&sensors.vision.targets.length>0){
                        Target t=sensors.vision.targets[0];out.forward=Math.abs(t.tx)<20?.3:0;out.turnClockwise=FieldLayout.clip(t.tx*.02,.3);
                    }else out.turnClockwise=.2;
                    break;
                case EXTRA_PICKUP:
                    if(age>=1.2||!movementOnly&&sensors.ballCount>=4){enter(State.WAITING);break;}
                    Outputs sweep=navigator.move(sensors.pose,extraSweep);
                    if(sweep==null){enter(State.WAITING);break;}
                    out=sweep;out.intakePower=1;out.pipeline=1;break;
                case ERROR:
                    if(recoverable&&retries++<3&&destination!=null)enter(State.MOVING);
                    else if(preParkPickup){preParkPickup=false;beginParking();}
                    else enter(State.WAITING);break;
                case WAIT_TILT:
                    // Bounded flight/settling time, not a camera-confirmed tilt.
                    if(routine==Routine.TEAM_TWO){if(age>=3.5)enter(State.PRE_PARK);break;}
                    int settled=hiveVision.state(now);
                    if(settled>=0&&settled!=finalShotHiveState){
                        flowerTiltConfirmed=true;flipped=settled==1;enter(State.WAITING);
                    }else if(age>5)fail("Flower Hive tilt not confirmed",false);
                    break;
                case WAITING:tryExtraPickup(sensors.ballCount);break;
                case PARK:
                    // Accept position BEFORE navigation can demand another turn.
                    // running=false latches zero outputs until an explicit new start.
                    if(sensors.pose.distance(destination)<=parkingToleranceInches){
                        parkingComplete=true;running=false;navigator.reset();break;
                    }
                    Outputs park=navigator.move(sensors.pose,destination);
                    if(park==null){parkingComplete=true;running=false;navigator.reset();}else out=park;break;
                default:break;
            }
        }catch(java.util.concurrent.CancellationException e){
            stop();return;
        }catch(IllegalStateException e){
            if(preParkPickup){preParkPickup=false;error="Optional pickup skipped: "+e.getMessage();beginParking();out=new Outputs();}
            else if(movementOnly&&state==State.MOVING&&!fallbackActive&&destination!=null){
                fallbackActive=true;navigator.useScriptedRoute(false,alliance);progressPose=currentPose;progressAt=now;
                error="Fixed path unavailable; safe replanning: "+e.getMessage();out=new Outputs();
            }
            else if(state==State.PARK){error="Parking route unavailable: "+e.getMessage();running=false;}
            else fail(e.getMessage(),true);
        }
        // State transitions cancel every actuator from the previous state in this same tick.
        if(!running||state==State.ERROR||state!=before)out=new Outputs();
        // Start intake while approaching a pickup spot, before the chassis pushes the balls away.
        if(running&&state==State.MOVING&&(afterMove==State.INTAKE||afterMove==State.EXTRA_PICKUP)&&sensors.ballCount<4&&sensors.pose.distance(destination)<24){out.intakePower=1;out.pipeline=1;}
        if(running&&state==State.MOVING&&afterMove==State.SHOOTING){out.shooterPower=power;if(Double.isNaN(shooterAt))shooterAt=now;}
        // Preserve state timers and chassis motion; never energize a mechanism in a dry run.
        if(movementOnly){out.shooterPower=out.feederPower=out.intakePower=0;out.pipeline=0;}
        if(io.stopRequested()){stop();return;}
        last=out;io.apply(out);
    }
}
