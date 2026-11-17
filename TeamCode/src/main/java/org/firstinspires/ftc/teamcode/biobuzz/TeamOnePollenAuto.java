package org.firstinspires.ftc.teamcode.biobuzz;

import java.util.List;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;

/** Team One: paced preloads, camera-guided Garden pickup, parking. */
public final class TeamOnePollenAuto implements RobotController {
    public enum State { SHOOTING, TO_GARDEN, TO_GARDEN_BACK, TO_GARDEN_RIGHT, GARDEN_ALIGN_HEADING, INTAKE_GARDEN, GARDEN_BACK_UP, GARDEN_RIGHT_SHIFT, GARDEN_RESTORE_HEADING, GARDEN_FORWARD_RETRY, TO_TEAM_TWO_SHOT, WAIT_FOR_TEAM_TWO_TAG, FOLLOW_UP_SHOOTING, OBSERVE_TILT, WAIT_TILT, CLEAR_PICKUP, SEARCH, APPROACH, CLOSE_PICKUP, REPOSITION, WAITING, PARK, STOPPED, ERROR }
    private final RobotIO io;
    private final FieldLayout field;
    private final Navigator nav;
    private HiveVision hive;
    private HiveShotGate shotGate;
    private boolean collecting,lastFeeding;
    private double feedElapsed,lastCommandAt;
    private State state=State.STOPPED;
    private Pose current,destination;
    private Pose gardenEntry;
    /** Start pose and robot-relative distances for the no-turn Garden transit. */
    private Pose gardenLegStart;
    private double gardenBackDistance,gardenRightDistance;
    /** Inward bypass used to keep the direct Garden-to-park path clear of the Flower. */
    private Pose parkVia;
    /** Alliance-wall corridor waypoint for the backup Garden-to-Team-Two shot route. */
    private Pose teamTwoShotVia;
    /** Initial sideways Garden exit before travelling the outside lane. */
    private Pose teamTwoShotExit;
    /** Shooting-lane stop between the Flower-safe waypoint and the wall shot. */
    private Pose teamTwoShotLane;
    private double gardenIntakeHeading;
    private int alliance;
    private boolean running,tiltConfirmed;
    private double power,startAt,stateAt,lastNow,spinAt;
    private String message="",error="";
    public double spinUpSeconds=1,feedSeconds=3,parkingToleranceInches=2,approachPower=.25;
    /** Measured forward stroke, not an assumed pickup count. Calibrate on the robot. */
    public double gardenAdvanceInches=18,gardenAdvanceSeconds=5,gardenAdvancePower=.18;
    /** Zero keeps legacy timed strokes; a positive value ends each stroke at this measured distance. */
    public double gardenPassDistanceInches=0,gardenTransitPower=.35,gardenRepositionPower=.25;
    public double gardenFirstPassSeconds=2,gardenRepositionSeconds=.45,gardenStagingOffsetInches=18,gardenIntakeHeadingRadians=Math.PI/2;
    /** Front-mounted camera: turn after wall clearance to view the Hive behind the shooter. */
    public double observationHeadingOffsetRadians=Math.PI;
    public boolean surveyBeforePickup=false;
    /** Practice only: allow a live tag frame without official Hive geometry. */
    public boolean allowAnyFreshAprilTag=false;
    /** Team 1 begins at a fixed, taped preload-shot pose; vision is not required to feed. */
    public boolean requireAprilTagForPreload=false;
    /** Backup mode: after two Garden passes, use Team Two's taped wall shot and fire only on a live tag. */
    public boolean useTeamTwoFollowUpShot=false;
    public int gardenPassLimit=0;
    /** Optional old spilled-pollen test; normal mode uses the Garden. */
    public boolean recoverPollenAfterTilt=false;
    private boolean survey;
    private double surveyAngle,surveyHeading,bestHeading,bestArea;
    private double avoidUntil,avoidStrafe;

    public TeamOnePollenAuto(RobotIO io,FieldLayout field){this.io=io;this.field=field;nav=new Navigator(field,io::stopRequested);}
    public void start(int alliance,double power){
        if(alliance<0||alliance>1||!Double.isFinite(power)||power<0||power>1
                ||!Double.isFinite(spinUpSeconds)||spinUpSeconds<0||!Double.isFinite(feedSeconds)||feedSeconds<=0
                ||!Double.isFinite(observationHeadingOffsetRadians)||!Double.isFinite(parkingToleranceInches)||parkingToleranceInches<=0||parkingToleranceInches>3
                ||!Double.isFinite(approachPower)||approachPower<=0||approachPower>.5
                ||!Double.isFinite(gardenAdvanceInches)||gardenAdvanceInches<=0||gardenAdvanceInches>24
                ||!Double.isFinite(gardenAdvanceSeconds)||gardenAdvanceSeconds<=0||gardenAdvanceSeconds>8
                ||!Double.isFinite(gardenAdvancePower)||gardenAdvancePower<=0||gardenAdvancePower>.6
                ||!Double.isFinite(gardenPassDistanceInches)||gardenPassDistanceInches<0||gardenPassDistanceInches>24
                ||!Double.isFinite(gardenTransitPower)||gardenTransitPower<=0||gardenTransitPower>.7
                ||!Double.isFinite(gardenRepositionPower)||gardenRepositionPower<=0||gardenRepositionPower>.6
                ||!Double.isFinite(gardenFirstPassSeconds)||gardenFirstPassSeconds<=0||gardenFirstPassSeconds>5
                ||!Double.isFinite(gardenRepositionSeconds)||gardenRepositionSeconds<=0||gardenRepositionSeconds>2
                ||!Double.isFinite(gardenStagingOffsetInches)||gardenStagingOffsetInches<0||gardenStagingOffsetInches>24
                ||!Double.isFinite(gardenIntakeHeadingRadians)||gardenPassLimit<0||gardenPassLimit>8)
            throw new IllegalArgumentException("Invalid Team One configuration");
        this.alliance=alliance;this.power=power;startAt=lastNow=io.nowSeconds();spinAt=Double.NaN;
        running=true;tiltConfirmed=survey=false;avoidUntil=0;current=destination=gardenEntry=parkVia=teamTwoShotExit=teamTwoShotVia=teamTwoShotLane=gardenLegStart=null;gardenBackDistance=gardenRightDistance=0;gardenIntakeHeading=Double.NaN;message=error="";
        collecting=lastFeeding=false;feedElapsed=0;lastCommandAt=startAt;
        shotGate=new HiveShotGate(field,alliance,false);
        hive=new HiveVision(field,alliance);nav.useScriptedRoute(true,alliance);enter(State.SHOOTING);io.apply(new Outputs());
    }
    private void enter(State next){
        state=next;stateAt=io.nowSeconds();nav.reset();
        if(next==State.TO_GARDEN)collecting=true;
        // The robot shoots with its rear toward the Hive.  When it reaches the
        // Garden, use the measured arrival heading as the intake heading rather
        // than commanding a field-fixed 90 degree turn.  This keeps the direct
        // diagonal trip and every retry pass aligned with the chassis direction
        // that actually brought it to the Garden.
        if(next==State.GARDEN_ALIGN_HEADING)
            gardenIntakeHeading=current!=null&&current.finite()
                    ? Pose.angle(current.heading) : Pose.angle(gardenIntakeHeadingRadians);
        if(next==State.TO_GARDEN_BACK||next==State.TO_GARDEN_RIGHT)gardenLegStart=current;
        if(next==State.INTAKE_GARDEN||next==State.GARDEN_FORWARD_RETRY)gardenEntry=current;
        if(next==State.FOLLOW_UP_SHOOTING){spinAt=Double.NaN;feedElapsed=0;lastFeeding=false;}
        if(next==State.PARK||next==State.STOPPED||next==State.ERROR)collecting=false;
        if(next==State.SEARCH&&surveyBeforePickup&&current!=null){survey=true;surveyAngle=0;surveyHeading=current.heading;bestHeading=Double.NaN;bestArea=0;}
    }
    public void stop(){running=false;state=State.STOPPED;io.apply(new Outputs());}
    public boolean isRunning(){return running;}
    public State getState(){return state;}
    public String lastError(){return error;}
    public String status(){return state+String.format(java.util.Locale.ROOT," · %.1f / 30 s · ",io.nowSeconds()-startAt)+message
            +(recoverPollenAfterTilt?(tiltConfirmed?" · Hive tilt confirmed":" · Tilt not yet confirmed"):" · Garden collection mode")+" · park by 26 s; stop at 30 s";}
    public Pose destination(){return running&&(state==State.PARK||moving())?destination:null;}
    public List<Pose> route(){return nav.route();}
    private boolean moving(){return state==State.TO_GARDEN||state==State.TO_GARDEN_BACK||state==State.TO_GARDEN_RIGHT||state==State.TO_TEAM_TWO_SHOT||state==State.OBSERVE_TILT||state==State.CLEAR_PICKUP||state==State.REPOSITION;}
    private boolean gardenMotion(){return state==State.TO_GARDEN||state==State.TO_GARDEN_BACK||state==State.TO_GARDEN_RIGHT||state==State.INTAKE_GARDEN||state==State.GARDEN_BACK_UP||state==State.GARDEN_RIGHT_SHIFT||state==State.GARDEN_RESTORE_HEADING||state==State.GARDEN_FORWARD_RETRY;}
    /**
     * Split the old diagonal translation into a backward leg and then a right
     * strafe.  The magnitudes are the same robot-relative components that the
     * diagonal command used, but neither leg ever asks the drivetrain to turn.
     */
    private void planGardenLegs(){
        double dx=destination.x-current.x,dy=destination.y-current.y;
        double c=Math.cos(current.heading),s=Math.sin(current.heading);
        gardenBackDistance=Math.abs(dx*c+dy*s);
        gardenRightDistance=Math.abs(dx*s-dy*c);
    }
    private boolean gardenLegDone(double targetInches){
        // Eight inches per second is deliberately conservative.  The timeout
        // guarantees that a momentary odometry dropout cannot strand the robot
        // before it starts the intake routine.
        return gardenLegStart!=null&&current!=null
                &&current.distance(gardenLegStart)>=Math.max(0,targetInches-1)
                || io.nowSeconds()-stateAt>=Math.max(.45,targetInches/8.0+.55);
    }
    private boolean gardenPassDone(double age){
        // Backup mode drives the existing stroke distance faster, rather than
        // increasing the distance merely because its motor power is higher.
        if(gardenPassDistanceInches>0&&gardenEntry!=null&&current!=null
                &&current.distance(gardenEntry)>=gardenPassDistanceInches)return true;
        return age>=gardenFirstPassSeconds;
    }
    private void followUpShotOrPark(){
        if(!useTeamTwoFollowUpShot){park();return;}
        collecting=false;
        destination=field.teamTwoWallHive[alliance];
        // Leave the Garden sideways first, then travel the outside lane. This
        // avoids the old diagonal that could clip either the Flower or Hive post.
        teamTwoShotExit=new Pose(alliance==0?-48:48,current.y,current.heading);
        teamTwoShotVia=new Pose(alliance==0?-48:48,alliance==0?47:-47,current.heading);
        teamTwoShotLane=new Pose(field.teamTwoWallHive[alliance].x,alliance==0?42:-42,current.heading);
        enter(State.TO_TEAM_TWO_SHOT);
        message="Garden passes complete; side exit, outside-lane retreat, shooting lane, then Team Two shot approach";
    }
    /**
     * Backup route: translate straight to Team Two's shot area without the
     * navigator's intermediate heading turns.  Only turn once after arriving,
     * so the robot cannot make small circles while crossing the field.
     */
    private Outputs directMoveToTeamTwoShot(){
        Pose target=teamTwoShotExit!=null?teamTwoShotExit:teamTwoShotVia!=null?teamTwoShotVia:teamTwoShotLane!=null?teamTwoShotLane:destination;
        double dx=target.x-current.x,dy=target.y-current.y;
        double distance=Math.hypot(dx,dy);
        // Once the staged route has reached the final wall-shot area, enter
        // the camera gate. Pinpoint can be a few inches different after the
        // Garden transit; holding this drive state for a perfect 2-inch pose
        // used to prevent a visible AprilTag from ever starting the shot.
        if(teamTwoShotExit==null&&teamTwoShotVia==null&&teamTwoShotLane==null&&distance<=4.0)return null;
        if(distance>2.0){
            double c=Math.cos(current.heading),s=Math.sin(current.heading);
            Outputs out=new Outputs();
            out.forward=FieldLayout.clip((dx*c+dy*s)*.055,.72);
            out.strafeRight=FieldLayout.clip((dx*s-dy*c)*.055,.72);
            double magnitude=Math.hypot(out.forward,out.strafeRight);
            if(magnitude>.72){out.forward*=.72/magnitude;out.strafeRight*=.72/magnitude;}
            return out;
        }
        // Stop at the Garden side-exit and outside-lane points without turning.
        if(teamTwoShotExit!=null){teamTwoShotExit=null;return new Outputs();}
        if(teamTwoShotVia!=null){teamTwoShotVia=null;return new Outputs();}
        // At the shooting-lane stop, turn before the final approach. The
        // robot's rear shooter (and rear-facing Hive view) is then pointed at
        // the Hive for the entire final straight move into the shot pose.
        if(teamTwoShotLane!=null){
            double laneHeadingError=Pose.angle(destination.heading-current.heading);
            if(Math.abs(laneHeadingError)>Math.toRadians(2)){
                Outputs out=new Outputs();
                out.turnClockwise=-FieldLayout.clip(Math.toDegrees(laneHeadingError)*.045,.35);
                return out;
            }
            teamTwoShotLane=null;
            message="Rear shooter faces Hive; make final straight approach to Team Two shot pose";
            return new Outputs();
        }
        double headingError=Pose.angle(destination.heading-current.heading);
        if(Math.abs(headingError)>Math.toRadians(2)){
            Outputs out=new Outputs();
            out.turnClockwise=-FieldLayout.clip(Math.toDegrees(headingError)*.045,.35);
            return out;
        }
        return null;
    }
    /** Robot-relative Team 1 has no color selection, so accept either alliance's Hive ID group at the taped wall pose. */
    private boolean correctTeamTwoTag(Sensors s,double now){
        // WAIT_FOR_TEAM_TWO_TAG is reached only after the same rear-facing
        // wall-shot alignment used by Team Two. Do not reject a live correct
        // tag because Pinpoint is a few inches off after the Garden route.
        if(!s.cameraConnected||!s.vision.fresh(now,0))return false;
        for(int[] group:field.nearTags)for(int id:group)for(Target target:s.vision.targets)if(target.id==id)return true;
        for(int[] group:field.farTags)for(int id:group)for(Target target:s.vision.targets)if(target.id==id)return true;
        return false;
    }
    private void park(){
        // After the backup's Team Two wall shot, use that role's nearby
        // parking half. Returning to Team One's starting-side park would add
        // an unnecessary full-field crossing after the second shot.
        Pose location=useTeamTwoFollowUpShot?field.teamTwoGardenSidePark[alliance]:field.teamOneStartSidePark[alliance];
        destination=location;
        // Move inward before travelling up the alliance lane. This keeps the
        // Garden-to-park diagonal outside the nearby Flower footprint.
        parkVia=useTeamTwoFollowUpShot?null:new Pose(alliance==0?-38:38,alliance==0?-16:16,current.heading);
        enter(State.PARK);message=useTeamTwoFollowUpShot?"Backup route: park in the nearby Team Two half; collection OFF":"Start-side-half parking via Flower-clear bypass; collection OFF";
    }
    private void fail(String reason){error=message=reason;running=false;state=State.ERROR;}
    private int pipeline(){return state==State.TO_GARDEN||state==State.TO_GARDEN_BACK||state==State.TO_GARDEN_RIGHT||state==State.INTAKE_GARDEN||state==State.GARDEN_BACK_UP||state==State.GARDEN_RIGHT_SHIFT||state==State.GARDEN_RESTORE_HEADING||state==State.GARDEN_FORWARD_RETRY||state==State.SEARCH||state==State.APPROACH||state==State.CLOSE_PICKUP||state==State.REPOSITION?1:0;}
    private Target pollen(Vision vision,double now){
        if(!vision.fresh(now,1))return null;
        Target best=null;
        for(Target t:vision.targets)if(t.id<0&&"yellow".equals(t.color)&&Double.isFinite(t.tx)&&Double.isFinite(t.areaPercent)&&t.areaPercent>0
                &&(best==null||t.areaPercent>best.areaPercent))best=t;
        return best;
    }
    private boolean roomToScan(Pose p){
        for(int i=0;i<24;i++)if(!field.allowed(new Pose(p.x,p.y,i*Math.PI/12)))return false;
        return true;
    }
    private void scanOrReposition(){
        if(roomToScan(current)){enter(State.SEARCH);return;}
        for(int radius=4;radius<=16;radius+=4)for(int i=0;i<8;i++){
            Pose q=new Pose(current.x+radius*Math.cos(i*Math.PI/4),current.y+radius*Math.sin(i*Math.PI/4),current.heading);
            if(roomToScan(q)&&field.clear(current,q,current.heading)){destination=q;enter(State.REPOSITION);return;}
        }
        message="No safe scanning clearance; waiting for parking";enter(State.WAITING);
    }
    /** Imported image-angle/area approach, with bounded cruise power and camera steering. */
    private Outputs approach(Target t){
        Outputs out=new Outputs();out.pipeline=1;out.intakePower=1;
        out.forward=approachPower;out.strafeRight=Math.abs(t.tx)<=2?0:FieldLayout.clip(t.tx*.008,.12);
        out.turnClockwise=FieldLayout.clip(t.tx*.018,.35);
        if(Math.abs(t.tx)>25){out.forward=0;out.turnClockwise=FieldLayout.clip(t.tx*.02,.25);}
        return guard(out);
    }
    private Outputs guard(Outputs out){
        if(state==State.APPROACH&&io.nowSeconds()<avoidUntil){out.forward=out.turnClockwise=0;out.strafeRight=avoidStrafe;}
        double dt=.12,h=Pose.angle(current.heading-out.turnClockwise*1.65*dt);
        Pose next=new Pose(current.x+(out.forward*Math.cos(h)+out.strafeRight*Math.sin(h))*25*dt,
                current.y+(out.forward*Math.sin(h)-out.strafeRight*Math.cos(h))*25*dt,h);
        if(!field.canRotate(current,h)||!field.clear(current,next,h)){
            if(state==State.APPROACH&&out.forward>0){
                for(double side:new double[]{-.3,.3}){
                    Pose lateral=new Pose(current.x+Math.sin(current.heading)*side*25*.75,current.y-Math.cos(current.heading)*side*25*.75,current.heading);
                    Pose ahead=new Pose(lateral.x+Math.cos(current.heading)*3,lateral.y+Math.sin(current.heading)*3,current.heading);
                    if(field.clear(current,lateral,current.heading)&&field.clear(lateral,ahead,current.heading)){
                        avoidUntil=io.nowSeconds()+.75;avoidStrafe=side;out.forward=out.turnClockwise=0;out.strafeRight=side;
                        message="Visible pollen; short collision-checked frame clearance";return out;
                    }
                }
            }
            out.forward=out.strafeRight=out.turnClockwise=out.intakePower=0;
            message="Camera approach blocked; holding rather than driving through obstacle";
            if(io.nowSeconds()-stateAt>1){enter(State.WAITING);}
        }
        return out;
    }
    public void tick(){
        Outputs out=new Outputs();
        if(!running){io.apply(out);return;}
        if(io.stopRequested()){stop();return;}
        try{
            double now=io.nowSeconds();
            if(!Double.isFinite(now)||now<lastNow){fail("Clock invalid");io.apply(out);return;}
            if(now-startAt>=30){running=false;message="30-second hard stop";io.apply(out);return;}
            Sensors s=io.readSensors();now=io.nowSeconds();
            if(io.stopRequested()){stop();return;}
            if(!Double.isFinite(now)||now<lastNow){fail("Clock invalid");io.apply(out);return;}lastNow=now;
            if(now-startAt>=30){running=false;message="30-second hard stop";io.apply(out);return;}
            // Once the front is in the Garden, keep the commanded straight intake
            // stroke alive even if Pinpoint briefly drops a sample. This phase has no
            // steering, turns, or odometry-based stop; four measured pieces or the
            // 26-second park deadline ends it.
            boolean keepDriving=gardenMotion()||state==State.PARK;
            // Team 1 starts at the taped shot pose. A transient Pinpoint update
            // loss must not abort a stationary, timed preload feed; departure to
            // the Garden still waits until at least one usable pose is available.
            // Both wall shots are stationary, timed camera-gated operations.
            // Once the backup has arrived at Team Two's wall shot area, a
            // transient Pinpoint stale sample must not abort its tag wait/feed
            // or keep it from reaching the normal parking fallback.
            boolean stationaryShot=state==State.SHOOTING||state==State.WAIT_FOR_TEAM_TWO_TAG||state==State.FOLLOW_UP_SHOOTING;
            boolean usablePose=s.pose!=null&&s.pose.finite()&&now-s.sampledAt>=0&&now-s.sampledAt<=.25;
            if(!keepDriving&&!stationaryShot&&(!s.odometryValid||!usablePose)){fail("Odometry unavailable or stale");io.apply(out);return;}
            if(usablePose)current=s.pose;
            else if(current==null&&!stationaryShot){fail("Odometry unavailable before a usable pose was recorded");io.apply(out);return;}
            if(lastFeeding)feedElapsed+=Math.min(.10,Math.max(0,now-lastCommandAt));
            lastFeeding=false;lastCommandAt=now;
            if(s.ballCount>=4)collecting=false;
            double elapsed=now-startAt;
            if(elapsed>=26&&state!=State.PARK)park();
            // Start earlier only when a distant search would consume the return-time reserve.
            if((state==State.TO_GARDEN||state==State.SEARCH||state==State.APPROACH||state==State.CLOSE_PICKUP||state==State.REPOSITION)
                    &&elapsed+current.distance(field.teamOneStartSidePark[alliance])*1.6/Math.max(1,25*field.drivePower)+1.2>=29)park();
            if((state==State.CLEAR_PICKUP||state==State.REPOSITION||state==State.SEARCH||state==State.APPROACH||state==State.CLOSE_PICKUP)&&!s.cameraConnected){
                message="Camera disconnected; recovery outputs OFF";out.pipeline=pipeline();io.apply(out);return;
            }
            if(pipeline()==0)hive.update(s.vision,now);
            State before=state;
            double age=now-stateAt;
            switch(state){
                case SHOOTING:
                    if(feedElapsed>=feedSeconds){
                        if(current==null){message="Preloads complete; waiting for a valid Pinpoint pose before Garden departure";break;}
                        if(!recoverPollenAfterTilt){
                            Pose g=field.garden[alliance];
                            // Keep the shot heading on the trip. Team 1 aligns to its
                            // intake heading only after it reaches the Garden.
                            destination=new Pose(g.x-gardenStagingOffsetInches*Math.cos(g.heading),g.y-gardenStagingOffsetInches*Math.sin(g.heading),current.heading);
                            enter(State.TO_GARDEN);message="Preloads complete; go straight to alliance Garden without a departure turn";break;
                        }
                        Pose start=field.hive[alliance];destination=new Pose(start.x,start.y-18*Math.sin(start.heading),Pose.angle(start.heading+observationHeadingOffsetRadians));
                        enter(State.OBSERVE_TILT);message="Preload feed attempt complete; move to tag-observation clearance";
                    }else{
                        Pose shot=field.hive[alliance];
                        boolean atShot=current.distance(shot)<=1&&Math.abs(Pose.angle(current.heading-shot.heading))<=Math.toRadians(2);
                        boolean practiceTagReady=s.cameraConnected&&s.vision.fresh(now,0)&&s.vision.targets.length>0;
                        boolean tagReady=allowAnyFreshAprilTag?practiceTagReady:(atShot&&shotGate.ready(s,now));
                        if(requireAprilTagForPreload&&!tagReady){
                            spinAt=Double.NaN;
                            message=allowAnyFreshAprilTag?"Shooting blocked: need any fresh AprilTag frame on pipeline 1":"Shooting blocked: need fresh centered raised own-Hive tags at start pose";
                            break;
                        }
                        if(Double.isNaN(spinAt))spinAt=now;
                        out.shooterPower=power;if(now-spinAt>=spinUpSeconds)out.feederPower=1;
                        message=requireAprilTagForPreload?(allowAnyFreshAprilTag?"Practice tag detected; paced preload feed":"Own raised Cell confirmed; paced preload feed"):"Fixed Team 1 start pose; paced preload feed";
                    }
                    break;
                case TO_GARDEN:
                    planGardenLegs();
                    enter(State.TO_GARDEN_BACK);
                    message="Garden transit: drive backward first, without turning";
                    break;
                case TO_GARDEN_BACK:
                    // The robot begins with its rear at the Hive.  Backing away
                    // is the first safe, non-turning leg toward the Garden.
                    out.forward=-gardenTransitPower;
                    if(gardenLegDone(gardenBackDistance)){
                        enter(State.TO_GARDEN_RIGHT);
                        message="Garden transit: backward leg complete; strafe right without turning";
                    }else message="Garden transit: backing up; hold heading";
                    break;
                case TO_GARDEN_RIGHT:
                    out.strafeRight=gardenTransitPower;
                    if(gardenLegDone(gardenRightDistance)){
                        enter(State.GARDEN_ALIGN_HEADING);
                        message="Garden reached; preserve arrival heading before intake";
                    }else message="Garden transit: strafing right; hold heading";
                    break;
                case GARDEN_ALIGN_HEADING:
                    double intakeHeadingError=Pose.angle(gardenIntakeHeading-current.heading);
                    if(Math.abs(intakeHeadingError)<=Math.toRadians(2)){enter(State.INTAKE_GARDEN);message="Garden arrival heading held; begin repeated forward intake passes";}
                    else out.turnClockwise=-FieldLayout.clip(Math.toDegrees(intakeHeadingError)*.05,.25);
                    break;
                case INTAKE_GARDEN:
                    // The contingency routine deliberately completes both
                    // physical strokes.  A noisy/full inventory reading must
                    // not cancel its second forward-and-intake pass.
                    if(s.ballCount>=4&&!useTeamTwoFollowUpShot){followUpShotOrPark();break;}
                    if(gardenPassDone(age)){
                        if(useTeamTwoFollowUpShot&&gardenPassLimit<=1){followUpShotOrPark();break;}
                        enter(State.GARDEN_BACK_UP);
                        message="Garden intake pass complete; back up slightly before the next pass";
                        break;
                    }
                    Target gardenBall=pollen(s.vision,now);
                    out.forward=gardenAdvancePower;out.intakePower=1;
                    if(gardenBall!=null){
                        out.strafeRight=Math.abs(gardenBall.tx)>6?FieldLayout.clip(gardenBall.tx*.012,.12):0;
                    }
                    // On the real robot, keep driving the intake forward until it has
                    // actually measured four pieces or the 26-second park command
                    // takes over. Do not use camera, simulated-clearance, distance,
                    // or timer cutoffs for this straight, no-turn collection phase.
                    double headingError=Double.isFinite(gardenIntakeHeading)?Pose.angle(gardenIntakeHeading-current.heading):0;
                    out.turnClockwise=-FieldLayout.clip(Math.toDegrees(headingError)*.05,.22);
                    message="Garden intake: hold first-pass heading; drive until four pieces or 26-second parking";
                    break;
                case GARDEN_BACK_UP:
                    out.forward=-gardenRepositionPower;out.intakePower=1;
                    if(age>=gardenRepositionSeconds){enter(State.GARDEN_RIGHT_SHIFT);message="Garden retry: shift right slightly";}
                    break;
                case GARDEN_RIGHT_SHIFT:
                    out.strafeRight=gardenRepositionPower;out.intakePower=1;
                    if(age>=gardenRepositionSeconds){enter(State.GARDEN_RESTORE_HEADING);message="Garden retry: restore first-pass heading";}
                    break;
                case GARDEN_RESTORE_HEADING:
                    double restoreError=Double.isFinite(gardenIntakeHeading)?Pose.angle(gardenIntakeHeading-current.heading):0;
                    if(Math.abs(restoreError)<=Math.toRadians(2)){enter(State.GARDEN_FORWARD_RETRY);message="Garden retry: original heading restored; drive into next pollen pair";}
                    else if(age>=2){enter(State.GARDEN_FORWARD_RETRY);message="Garden retry: heading-restore timeout; continue carefully";}
                    else out.turnClockwise=-FieldLayout.clip(Math.toDegrees(restoreError)*.05,.25);
                    break;
                case GARDEN_FORWARD_RETRY:
                    out.forward=gardenAdvancePower;out.intakePower=1;
                    Target retryBall=pollen(s.vision,now);
                    if(retryBall!=null)out.strafeRight=Math.abs(retryBall.tx)>6?FieldLayout.clip(retryBall.tx*.012,.12):0;
                    double retryHeadingError=Double.isFinite(gardenIntakeHeading)?Pose.angle(gardenIntakeHeading-current.heading):0;
                    out.turnClockwise=-FieldLayout.clip(Math.toDegrees(retryHeadingError)*.05,.22);
                    if((useTeamTwoFollowUpShot?gardenPassDone(age):age>=.60)){
                        if(useTeamTwoFollowUpShot&&gardenPassLimit<=2)followUpShotOrPark();
                        else if(useTeamTwoFollowUpShot){enter(State.GARDEN_BACK_UP);message="Garden pass complete; reset for the next measured intake pass";}
                        else {enter(State.INTAKE_GARDEN);message="Garden retry complete; repeat forward intake cycle until four pieces or parking";}
                    }
                    break;
                case TO_TEAM_TWO_SHOT:
                    out=directMoveToTeamTwoShot();
                    if(out==null){enter(State.WAIT_FOR_TEAM_TWO_TAG);message="At Team Two wall shot; waiting for a fresh AprilTag before follow-up feed";}
                    break;
                case WAIT_FOR_TEAM_TWO_TAG:
                    if(correctTeamTwoTag(s,now)){
                        enter(State.FOLLOW_UP_SHOOTING);
                        message="Correct Hive AprilTag detected; begin paced follow-up feed";
                    }else message="At Team Two wall shot; waiting for a current Hive AprilTag";
                    break;
                case FOLLOW_UP_SHOOTING:
                    if(feedElapsed>=feedSeconds){park();break;}
                    if(Double.isNaN(spinAt))spinAt=now;
                    out.shooterPower=power;
                    if(now-spinAt>=spinUpSeconds)out.feederPower=1;
                    message="AprilTag confirmed at shot start; paced Team Two-position follow-up feed";
                    break;
                case OBSERVE_TILT:
                case CLEAR_PICKUP:
                case REPOSITION:
                    out=nav.move(current,destination);
                    if(out==null){
                        out=new Outputs();
                        if(state==State.OBSERVE_TILT){enter(State.WAIT_TILT);message="Observe own Hive: require stable opposite Cell up";}
                        else scanOrReposition();
                    }
                    break;
                case WAIT_TILT:
                    message=s.cameraConnected?"Await fresh, calibrated own-Hive tilt evidence":"Limelight unavailable; wait safely for parking";
                    if(s.cameraConnected&&hive.state(now)==1){
                        tiltConfirmed=true;destination=new Pose(current.x,current.y-8*Math.signum(field.hive[alliance].y),current.heading);
                        enter(State.CLEAR_PICKUP);message="Hive tilt confirmed; back off for front-camera pollen view";
                    }
                    break;
                case SEARCH:
                case APPROACH:
                case CLOSE_PICKUP:
                    if(s.ballCount>=4){enter(State.WAITING);message="Measured inventory full; wait for parking";break;}
                    if(!s.cameraConnected){enter(State.SEARCH);message="Camera disconnected; drive/intake OFF";break;}
                    if(state==State.SEARCH&&survey){
                        if(!roomToScan(current)){scanOrReposition();break;}
                        surveyAngle+=Math.abs(Pose.angle(current.heading-surveyHeading));surveyHeading=current.heading;
                        Target seen=pollen(s.vision,now);
                        if(seen!=null&&seen.areaPercent>bestArea){bestArea=seen.areaPercent;bestHeading=Pose.angle(current.heading-Math.toRadians(seen.tx));}
                        if(surveyAngle<2*Math.PI-.08){message="Survey all camera bearings for the closest-looking pollen";out.turnClockwise=.65;out=guard(out);break;}
                        if(Double.isFinite(bestHeading)){
                            double error=Pose.angle(bestHeading-current.heading);
                            if(Math.abs(error)>Math.toRadians(4)){message="Reacquire best observed pollen bearing";out.turnClockwise=-FieldLayout.clip(Math.toDegrees(error)*.025,.65);out=guard(out);break;}
                        }
                        survey=false;stateAt=now;message="Survey complete; require a fresh visible pollen target";break;
                    }
                    Target target=pollen(s.vision,now);
                    if(target==null){
                        enter(State.WAITING);message="No fresh pollen target; hold still until parking";
                    }else if(state==State.CLOSE_PICKUP){
                        out.intakePower=1;
                        if(age>=.7){
                            Pose q=new Pose(current.x-4*Math.cos(current.heading),current.y-4*Math.sin(current.heading),current.heading);
                            if(field.clear(current,q,current.heading)){destination=q;enter(State.REPOSITION);}
                            else{enter(State.WAITING);message="Close pickup attempt complete; no safe retreat";}
                        }
                    }else if(target.areaPercent>=12){enter(State.CLOSE_PICKUP);message="Pollen close; bounded intake attempt (count remains unknown)";}
                    else{if(state==State.SEARCH)enter(State.APPROACH);message="Live largest-pollen camera approach";out=approach(target);}
                    break;
                case WAITING:break;
                case PARK:
                    Pose parkingGoal=parkVia==null?destination:parkVia;
                    if(parkVia!=null&&current.distance(parkVia)<=3){parkVia=null;out=new Outputs();message="Flower-clear bypass reached; continue to configured parking target";break;}
                    double parkHeadingError=Pose.angle(parkingGoal.heading-current.heading);
                    if(parkVia==null&&current.distance(destination)<=parkingToleranceInches&&Math.abs(parkHeadingError)<=Math.toRadians(3)){running=false;message="Parked at configured location and heading: all outputs OFF";break;}
                    // Garden collection may have carried the robot outside of the
                    // simulator's modeled corridor. At the 26-second deadline use a
                    // simple field-vector drive toward parking instead of invoking
                    // the collision-route planner, which can reject that pose and
                    // throw a runtime exception. Correct toward the configured
                    // parking heading before the final few inches.
                    double dx=parkingGoal.x-current.x,dy=parkingGoal.y-current.y;
                    double c=Math.cos(current.heading),sine=Math.sin(current.heading);
                    if(parkVia==null&&current.distance(destination)<=4&&Math.abs(parkHeadingError)>Math.toRadians(3)){
                        out.forward=out.strafeRight=0;
                    }else{
                        out.forward=FieldLayout.clip((dx*c+dy*sine)*.08,.45);
                        out.strafeRight=FieldLayout.clip((dx*sine-dy*c)*.08,.45);
                    }
                    out.turnClockwise=-FieldLayout.clip(Math.toDegrees(parkHeadingError)*.05,.25);
                    message=parkVia==null?"Parking direct to configured location and heading":"Parking via Flower-clear bypass";
                    break;
                default:break;
            }
            if(state!=before||!running)out=new Outputs();
            boolean backupGardenStroke=useTeamTwoFollowUpShot&&(state==State.INTAKE_GARDEN||state==State.GARDEN_BACK_UP||state==State.GARDEN_RIGHT_SHIFT||state==State.GARDEN_RESTORE_HEADING||state==State.GARDEN_FORWARD_RETRY);
            out.intakePower=running&&state!=State.PARK&&(backupGardenStroke||(collecting&&s.ballCount<4))?1:0;
            lastFeeding=out.feederPower>0;
            out.pipeline=pipeline();
            if(io.stopRequested()){stop();return;}
            io.apply(out);
        }catch(java.util.concurrent.CancellationException e){stop();}
        catch(RuntimeException e){
            // The backup route must keep its two-pass sequence alive even if a
            // transient odometry/navigation sample fails.  It retries on the
            // next loop and still enters the normal 26-second parking fallback.
            if(gardenMotion()||state==State.PARK||useTeamTwoFollowUpShot){message="Transient backup/Garden fault; retrying next loop";io.apply(new Outputs());}
            else{fail("Team One stopped safely: "+e.getMessage());io.apply(new Outputs());}
        }
    }
}
