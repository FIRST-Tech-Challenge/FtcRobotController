package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.biobuzz.FieldLayout;
import org.firstinspires.ftc.teamcode.biobuzz.AutoDebugSnapshot;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO;
import org.firstinspires.ftc.teamcode.biobuzz.RobotController;
import org.firstinspires.ftc.teamcode.biobuzz.TeamOnePollenAuto;
import org.firstinspires.ftc.teamcode.biobuzz.TeamTwoHiveAuto;
import org.firstinspires.ftc.teamcode.quantumstorm.Vision.TeamOneLimelight;
import org.firstinspires.ftc.teamcode.biobuzz.RobotIO.*;
import org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine.Routine;
import org.firstinspires.ftc.teamcode.biobuzz.ftc.SafeShutdown;
import org.firstinspires.ftc.teamcode.quantumstorm.Constants;
import org.firstinspires.ftc.teamcode.quantumstorm.DriveChain.MecanumDrive;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.FlowerIntake;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PinpointOdometry;
import org.firstinspires.ftc.teamcode.quantumstorm.Sensors.PollenInventory;

/** Team One recovers pollen; Team Two gates opposite-wall shots on live own-Hive tags.
 * Calibrate start, Pinpoint offsets/directions, waypoints and shooter power.
 * Unknown inventory uses bounded attempts, never invented ball counts.
 */
public final class AutoStateMachine implements AutoCloseable {
    private final LinearOpMode opMode;
    private final AutoStartPosition position;
    private final MecanumDrive drive;
    private final PinpointOdometry odometry;
    private final Shooter shooter;
    private final Intake intake;
    private final FlowerIntake flowerIntake;
    private final ElapsedTime clock=new ElapsedTime();
    private final RobotController routine;
    private final TeamOneLimelight camera;
    private final PollenInventory inventory;
    private final RobotIO robotIO;
    private Sensors debugSensors;
    private Vision debugRawVision,debugLastTags;
    private Outputs debugOutputs=new Outputs();
    private String debugReadError="";
    private boolean closed;
    private final boolean robotRelative;
    private final boolean teamOneBackup;
    private final org.firstinspires.ftc.teamcode.biobuzz.MirroredHiveTags mirroredTags;

    /** Canonical frame is re-anchored at the same physical role start on either alliance. */
    public AutoStateMachine(LinearOpMode opMode,boolean teamTwo){
        this(opMode,teamTwo,false);
    }
    /** Team One contingency: two Garden strokes, teammate's wall shot, then park. */
    public AutoStateMachine(LinearOpMode opMode,boolean teamTwo,boolean teamOneBackup){
        this(opMode,teamTwo?AutoStartPosition.RED_2:AutoStartPosition.RED_1,teamTwo?Routine.TEAM_TWO:Routine.TEAM_ONE,true,teamOneBackup);
    }
    private String label(){return robotRelative?(teamOneBackup?"Team 1 Backup Auto":position==AutoStartPosition.RED_2?"Team 2 Auto":"Team 1 Auto")+" (robot-relative)":position.label;}

    public AutoStateMachine(LinearOpMode opMode,AutoStartPosition position) {
        this(opMode,position,position==AutoStartPosition.RED_2||position==AutoStartPosition.BLUE_2?Routine.TEAM_TWO:Routine.TEAM_ONE);
    }
    public AutoStateMachine(LinearOpMode opMode,AutoStartPosition position,Routine selectedRoutine) {
        this(opMode,position,selectedRoutine,false,false);
    }
    private AutoStateMachine(LinearOpMode opMode,AutoStartPosition position,Routine selectedRoutine,boolean robotRelative,boolean teamOneBackup) {
        this.opMode=opMode;this.position=position;this.robotRelative=robotRelative;this.teamOneBackup=teamOneBackup;
        int alliance=position==AutoStartPosition.RED_1||position==AutoStartPosition.RED_2?0:1;
        FieldLayout field=new FieldLayout();Pose start=field.roleStart(alliance,selectedRoutine==Routine.TEAM_TWO);
        mirroredTags=robotRelative&&selectedRoutine!=Routine.FULL?new org.firstinspires.ftc.teamcode.biobuzz.MirroredHiveTags(field,selectedRoutine==Routine.TEAM_TWO):null;
        initStage("Initializing drive motors");
        drive=new MecanumDrive(opMode.hardwareMap);
        initStage("Initializing odometry_1: verify Pinpoint wiring/configuration");
        odometry=new PinpointOdometry(opMode.hardwareMap,new Pose2D(DistanceUnit.INCH,start.x,start.y,AngleUnit.RADIANS,start.heading));
        initStage("Initializing shooter and intake; OFF until START");
        shooter=new Shooter(opMode.hardwareMap);intake=new Intake(opMode.hardwareMap);
        initStage("Initializing flower_servo: arm LIFTS now");
        flowerIntake=new FlowerIntake(opMode.hardwareMap);flowerIntake.lift();
        field.upperTagHeight=selectedRoutine==Routine.TEAM_TWO?Constants.TEAM_TWO_UPPER_TAG_HEIGHT_IN:Constants.TEAM_ONE_UPPER_TAG_HEIGHT_IN;
        field.lowerTagHeight=selectedRoutine==Routine.TEAM_TWO?Constants.TEAM_TWO_LOWER_TAG_HEIGHT_IN:Constants.TEAM_ONE_LOWER_TAG_HEIGHT_IN;
        initStage("Initializing Limelight: own raised Cell required before feeding");
        camera=selectedRoutine==Routine.FULL?null:new TeamOneLimelight(opMode.hardwareMap,clock::seconds,selectedRoutine==Routine.TEAM_TWO);
        inventory=selectedRoutine==Routine.FULL?null:new PollenInventory(opMode.hardwareMap);
        robotIO=new RobotIO(){
            private boolean stopped;
            public boolean stopRequested(){return opMode.isStopRequested()||Thread.currentThread().isInterrupted();}
            public double nowSeconds(){return clock.seconds();}
            public Sensors readSensors(){
                odometry.update();double now=clock.seconds();
                Pose pose=new Pose(odometry.getX(),odometry.getY(),odometry.getPose().getHeading(AngleUnit.RADIANS));
                Vision vision=camera==null?Vision.empty(0,now):camera.read(now);
                debugRawVision=vision;
                if(vision.fresh(now,0)&&vision.targets.length>0)debugLastTags=vision;
                if(mirroredTags!=null){
                    Pose shot=selectedRoutine==Routine.TEAM_TWO?field.teamTwoWallHive[0]:field.hive[0];
                    boolean atShot=pose.distance(shot)<=1&&Math.abs(Pose.angle(pose.heading-shot.heading))<=Math.toRadians(2);
                    Vision normalized=mirroredTags.normalize(vision,now,atShot);
                    // Practice still feeds from any fresh tag, but keep the strict
                    // detector alive for telemetry so Red/Blue family diagnosis is
                    // possible before the match switch is turned off.
                    // The backup routine deliberately uses the Team Two wall
                    // pose after a Team One start.  Its follow-up shot needs a
                    // simple live-tag confirmation there, not the Team One
                    // start-pose family filter.
                    // Team Two and the backup use their own direct far-Hive ID
                    // check at the Team Two wall pose. Do not discard those raw
                    // IDs merely because the optional family diagnostic is not
                    // yet locked.
                    if(!Constants.PRACTICE_ALLOW_ANY_FRESH_APRILTAG&&!teamOneBackup&&selectedRoutine!=Routine.TEAM_TWO)vision=normalized;
                }
                PollenInventory.Sample count=inventory==null?null:inventory.read();
                debugSensors=new Sensors(pose,now,odometry.isValid(),camera!=null&&camera.connected(),count==null?-1:count.total,count==null?-1:count.pollen,vision);
                debugReadError="";return debugSensors;
            }
            public void apply(Outputs o){
                debugOutputs=o.copy();
                if(stopRequested()||o.forward==0&&o.strafeRight==0&&o.turnClockwise==0&&o.shooterPower==0&&o.feederPower==0&&o.intakePower==0){
                    if(!stopped){stopped=true;SafeShutdown.run(drive::stop,shooter::stop,intake::stop);}
                    // Stop actuators BEFORE any camera SDK call, including a pipeline transition.
                    if(camera!=null&&!stopRequested())camera.select(o.pipeline);
                    return;
                }
                stopped=false;
                double[] wheels=o.wheelPowers(Constants.AUTO_FORWARD_POWER_SIGN);drive.setPowers(wheels[0],wheels[1],wheels[2],wheels[3]);
                shooter.apply(o.shooterPower,o.feederPower);intake.setPower(o.intakePower*Constants.INTAKE_POWER);
                if(camera!=null&&!stopRequested())camera.select(o.pipeline);
            }
        };
        RobotIO io=robotIO;
        opMode.telemetry.setMsTransmissionInterval(100);
        if(selectedRoutine==Routine.TEAM_ONE){
            TeamOnePollenAuto one=new TeamOnePollenAuto(io,field);
            one.spinUpSeconds=Constants.AUTO_SHOOTER_SPINUP_SEC;one.feedSeconds=Constants.AUTO_TEAM_ONE_FEED_TIME_SEC;
            one.observationHeadingOffsetRadians=Math.toRadians(Constants.TEAM_ONE_TAG_VIEW_TURN_DEG);
            one.surveyBeforePickup=false;
            one.gardenAdvanceInches=Constants.AUTO_GARDEN_ADVANCE_IN;
            one.gardenAdvanceSeconds=Constants.AUTO_GARDEN_ADVANCE_SEC;
            one.gardenAdvancePower=Constants.AUTO_GARDEN_ADVANCE_POWER;
            one.gardenFirstPassSeconds=Constants.AUTO_GARDEN_FIRST_PASS_SEC;
            one.gardenIntakeHeadingRadians=Math.toRadians(Constants.TEAM_ONE_GARDEN_INTAKE_HEADING_DEG);
            one.allowAnyFreshAprilTag=Constants.PRACTICE_ALLOW_ANY_FRESH_APRILTAG;
            one.useTeamTwoFollowUpShot=teamOneBackup;
            one.gardenPassLimit=teamOneBackup?2:0;
            // The contingency route crosses to the opposite wall after Garden.
            // Keep enough time for four paced shots while reserving its nearby
            // Team Two parking half before the fixed 26-second park cutoff.
            if(teamOneBackup){
                one.spinUpSeconds=.60;
                // Four pollen need at least four paced index advances.  Keep a
                // full 3.6-second feed window before either Garden departure
                // or the follow-up-shot park transition.
                one.feedSeconds=3.60;
                // Two longer measured strokes sweep farther into the Garden,
                // then the backup immediately starts its Team Two shot route.
                one.gardenTransitPower=.55;
                one.gardenRepositionPower=.38;
                one.gardenAdvancePower=.42;
                one.gardenPassDistanceInches=16.0;
                one.gardenFirstPassSeconds=2.0; // timeout only if odometry drops out
                one.gardenRepositionSeconds=.18;
                // Stop only six inches short of the Garden.  The two short
                // forward intake strokes cover the remainder, avoiding the
                // old long retreat after the preload shot.
                one.gardenStagingOffsetInches=6.0;
            }
            routine=one;
        }else if(selectedRoutine==Routine.TEAM_TWO){
            TeamTwoHiveAuto two=new TeamTwoHiveAuto(io,field);
            two.spinUpSeconds=Constants.AUTO_SHOOTER_SPINUP_SEC;two.feedSeconds=Constants.AUTO_TEAM_TWO_FEED_TIME_SEC;
            two.allowAnyFreshAprilTag=Constants.PRACTICE_ALLOW_ANY_FRESH_APRILTAG;
            routine=two;
        }else{
            org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine two=new org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine(io,field,false,selectedRoutine,false,false);
            two.parkingToleranceInches=2.0;two.waitUntilParkTime=true;routine=two;
        }
        opMode.telemetry.addData("Autonomous",label());
        if(robotRelative)opMode.telemetry.addLine("No alliance selection: place at the matching Team 1/2 start on either side. Coordinates use a local route reference, not absolute field color.");
        opMode.telemetry.addData(robotRelative?"Local route reference":"Required start","X %.3f, Y %.3f in, heading %.1f deg",start.x,start.y,Math.toDegrees(start.heading));
        opMode.telemetry.addData("Autonomous role",selectedRoutine);
        opMode.telemetry.addData("Drive forward power sign",Constants.AUTO_FORWARD_POWER_SIGN);
        opMode.telemetry.addLine("Start FRONT toward wall, rear shooter toward Hive. Retreat BACKWARD into the field after shooting.");
        opMode.telemetry.addLine(camera==null?"Legacy camera-free mode":camera.status());
        opMode.telemetry.addLine(selectedRoutine==Routine.TEAM_TWO?"Team 2: START at opposite-wall shot beside Flower, FRONT to wall / REAR to Hive. Wait for raised far tags -> feed -> Flower -> wall -> verify/feed -> park.":teamOneBackup?"Team 1 backup: preloads -> two Garden intake passes -> Team 2 wall shot on fresh AprilTag -> park by 26 s.":"Team 1: preloads -> Garden pollen intake -> start-side-half park by 26 s.");
        opMode.telemetry.addLine("LIVE shooter/feed/intake after START. Load four pollen; clear the robot and firing path.");
        opMode.telemetry.addLine(inventory==null?"Inventory unknown: timed attempts do not invent ball counts.":inventory.status());
        opMode.telemetry.addLine(selectedRoutine==Routine.TEAM_TWO?(Constants.PRACTICE_ALLOW_ANY_FRESH_APRILTAG?"PRACTICE TAG MODE: any fresh Pipeline 1 AprilTag permits Team 2 timed feeding.":"Pipeline 1: raised own far Cell required before AND during feed; no any-tag fallback."):camera==null?"No extra pickup; wait then park at 26 s; stop at 30 s.":Constants.PRACTICE_ALLOW_ANY_FRESH_APRILTAG?"PRACTICE TAG MODE: any fresh Pipeline 1 AprilTag permits Team 1 preload feeding.":"Team One: raised own near Cell tags required before AND during feeding; yellow-only Garden pickup.");
        opMode.telemetry.addLine("Intake ON from pickup departure until four measured pieces or parking. Unknown counts are not assumed full.");
        opMode.telemetry.addLine("No idle camera-search spins. Garden: bounded slow forward intake stroke, then stationary wait for parking.");
        if(selectedRoutine==Routine.TEAM_TWO){Pose park=field.teamTwoGardenSidePark[alliance];opMode.telemetry.addData("Team Two shooting-side-half park","X %.1f, Y %.1f in",park.x,park.y);}
        if(selectedRoutine==Routine.TEAM_ONE){Pose park=field.teamOneStartSidePark[alliance];opMode.telemetry.addData("Team One start-side-half park","X %.1f, Y %.1f in; nearest starting wall",park.x,park.y);}
        if(selectedRoutine==Routine.TEAM_TWO){Pose shot=field.teamTwoWallHive[alliance];opMode.telemetry.addData("Team Two wall shot","X %.2f, Y %.2f in, heading %.1f deg",shot.x,shot.y,Math.toDegrees(shot.heading));opMode.telemetry.addLine("Limelight must see the Hive behind the rear shooter while the front faces the wall.");}
        opMode.telemetry.addLine("Parking: within 2 inches -> all outputs OFF; no final heading correction.");
        opMode.telemetry.addLine("Calibrate start, odometry and drive speed; physical arrival times may differ.");
        opMode.telemetry.update();
    }

    private void initStage(String message){
        if(opMode.isStopRequested()||Thread.currentThread().isInterrupted())throw new java.util.concurrent.CancellationException("INIT cancelled");
        opMode.telemetry.addData("Startup",message);opMode.telemetry.update();
        com.qualcomm.robotcore.util.RobotLog.ii("BioBuzzInit",message);
    }

    private String lastError(){
        if(routine instanceof TeamOnePollenAuto)return ((TeamOnePollenAuto)routine).lastError();
        if(routine instanceof TeamTwoHiveAuto)return ((TeamTwoHiveAuto)routine).lastError();
        return ((org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine)routine).lastError();
    }

    /** Keep the Flower slider down from the post-shot retreat through pollen intake. */
    private void updateFlowerIntakeArm(){
        if(!(routine instanceof TeamTwoHiveAuto)){flowerIntake.lift();return;}
        TeamTwoHiveAuto.State state=(TeamTwoHiveAuto.State)routine.getState();
        switch(state){
            case RETREAT_FROM_HIVE:
            case ALIGN_FLOWER:
            case TO_FLOWER:
            case PRESS_FLOWER:
            case CLEAR_FLOWER:
            case TO_OUTLET_LANE:
            case TO_PICKUP:
            case INTAKE:
                flowerIntake.drop();
                break;
            default:
                flowerIntake.lift();
                break;
        }
    }

    /** Samples sensors during INIT without ticking autonomous or commanding motors. */
    public void updateInitTelemetry(){
        if(opMode.isStopRequested()||Thread.currentThread().isInterrupted())return;
        try{robotIO.readSensors();}
        catch(java.util.concurrent.CancellationException e){throw e;}
        catch(RuntimeException e){debugSensors=null;debugRawVision=null;debugReadError=e.getMessage()==null?e.getClass().getSimpleName():e.getMessage();}
        opMode.telemetry.addData("Autonomous",label());
        opMode.telemetry.addData("State","INIT - keep robot still for odometry calibration");
        addDebugTelemetry();opMode.telemetry.update();
    }
    private void addDebugTelemetry(){
        double now=clock.seconds();
        opMode.telemetry.addData("Limelight device",camera==null?"Not used (legacy)":camera.mapped()?"DETECTED: "+camera.hardwareName():camera.hardwareName());
        opMode.telemetry.addData("Limelight connection",debugSensors==null?"Not sampled":debugSensors.cameraConnected?"CONNECTED":"DISCONNECTED / NOT FOUND");
        if(camera!=null){
            opMode.telemetry.addData("Limelight status",camera.cachedStatus());
            opMode.telemetry.addData("Limelight requested pipeline",camera.selectedPipeline());
        }
        opMode.telemetry.addData("Raw AprilTag IDs",AutoDebugSnapshot.ids(debugRawVision,now));
        opMode.telemetry.addData("Raw tag measurements",AutoDebugSnapshot.details(debugRawVision,now));
        opMode.telemetry.addData("Last tag frame age",AutoDebugSnapshot.age(debugLastTags,now));
        if(teamOneBackup){
            boolean liveBackupTag=debugSensors!=null&&debugSensors.cameraConnected&&debugSensors.vision.fresh(now,0)&&debugSensors.vision.targets.length>0;
            opMode.telemetry.addData("Backup shot tag permission",liveBackupTag?"LIVE frame: checking far-Hive ID and shot pose":"WAITING: point at tag / verify AprilTag pipeline 1");
        }
        opMode.telemetry.addData("Controller tag IDs",AutoDebugSnapshot.ids(debugSensors==null?null:debugSensors.vision,now));
        if(teamOneBackup)opMode.telemetry.addData("Hive tag family","Not used by backup: any current AprilTag authorizes the Team 2 follow-up shot");
        else if(routine instanceof TeamTwoHiveAuto)opMode.telemetry.addData("Hive tag family","Diagnostic only: Team 2 requires a current Hive AprilTag at the wall shot pose");
        else if(mirroredTags!=null)opMode.telemetry.addData("Hive tag family",mirroredTags.observedFamily()<0?"Not locked - need 2 correct raised tags, centered at shot pose":(Constants.PRACTICE_ALLOW_ANY_FRESH_APRILTAG?"Practice bypass; detector sees ":"")+(mirroredTags.observedFamily()==0?"Red family":"Blue family"));
        opMode.telemetry.addData("Current location",AutoDebugSnapshot.pose(debugSensors));
        opMode.telemetry.addData("Coordinate frame",robotRelative?"LOCAL role reference; inches; heading CCW from +X. Not absolute alliance coordinates.":"Field X/Y inches; heading CCW from +X");
        opMode.telemetry.addData("Odometry sample",debugSensors!=null&&debugSensors.odometryValid?"VALID":"INVALID / UNAVAILABLE - do not trust last known pose");
        Pose target=routine.destination();
        opMode.telemetry.addData("Destination",target==null?"None - stationary state":String.format(java.util.Locale.US,"X %.1f, Y %.1f in, heading %.1f deg",target.x,target.y,Math.toDegrees(target.heading)));
        if(target!=null&&debugSensors!=null)opMode.telemetry.addData("Distance to target (in)","%.1f",debugSensors.pose.distance(target));
        opMode.telemetry.addData("Drive request F / right / CW","%.2f / %.2f / %.2f",debugOutputs.forward,debugOutputs.strafeRight,debugOutputs.turnClockwise);
        opMode.telemetry.addData("Shooter power / spindexer / intake","%.2f / %.2f / %.2f",debugOutputs.shooterPower,debugOutputs.feederPower,debugOutputs.intakePower);
        opMode.telemetry.addData("Shooter applied / requested power","%.2f / %.2f",shooter.getAppliedPower(),shooter.getTargetPower());
        opMode.telemetry.addData("Flower intake","%s %.0f deg",flowerIntake.isDown()?"DOWN":"UP",flowerIntake.getAngle());
        opMode.telemetry.addData("Measured pieces",debugSensors==null||debugSensors.ballCount<0?"UNKNOWN":debugSensors.ballCount+" / 4");
        opMode.telemetry.addData("Measured pollen",debugSensors==null||debugSensors.pollenCount<0?"UNKNOWN":debugSensors.pollenCount+" / 4");
        if(!debugReadError.isEmpty())opMode.telemetry.addData("Sensor read error",debugReadError);
        odometry.addTelemetry(opMode.telemetry);
    }

    public void run(){
        int alliance=position==AutoStartPosition.RED_1||position==AutoStartPosition.RED_2?0:1;
        if(opMode.isStopRequested())return;
        clock.reset();
        debugSensors=null;debugRawVision=debugLastTags=null;debugOutputs=new Outputs();debugReadError="";
        try {
            if(camera!=null)camera.reset();
            if(mirroredTags!=null)mirroredTags.reset();
            routine.start(alliance,Constants.SHOOTER_POWER);
            // Bounded, stationary wait; included in the match's 26/30-second clock.
            while(opMode.opModeIsActive()&&clock.seconds()<2){
                odometry.update();
                opMode.telemetry.addData("Startup","Waiting for Pinpoint: %s",odometry.getStatus());opMode.telemetry.update();
                if(odometry.getStatus().name().equals("READY")||odometry.getStatus().name().startsWith("FAULT"))break;
                opMode.idle();
            }
            while(opMode.opModeIsActive()&&routine.isRunning()){
                routine.tick();
                updateFlowerIntakeArm();
                opMode.telemetry.addData("Autonomous",label());
                opMode.telemetry.addData("Routine",routine.status());
                opMode.telemetry.addData("State",routine.getState());
                opMode.telemetry.addData("Last error",lastError().isEmpty()?"None":lastError());
                opMode.telemetry.addData("Navigation",routine instanceof TeamTwoHiveAuto?"Opposite-wall shots; fixed route; fresh own-tag gate":camera==null?"Fixed simulator route; no route search":"Camera pollen approach + collision-checked start-side-half park");
                opMode.telemetry.addData("Mechanisms",inventory==null?"LIVE hardware outputs; unknown inventory":inventory.status());
                addDebugTelemetry();opMode.telemetry.update();
                opMode.idle();
            }
            // A controller failure previously ended the LinearOpMode immediately, so
            // the Driver Hub only flashed the useful reason before preselecting TeleOp.
            // Keep the robot stopped and the diagnostics visible until the operator
            // presses STOP. Normal completion still exits normally.
            if(opMode.opModeIsActive()&&!lastError().isEmpty()){
                while(opMode.opModeIsActive()&&!opMode.isStopRequested()){
                    opMode.telemetry.addData("Autonomous","ERROR - robot outputs are OFF");
                    opMode.telemetry.addData("Last error",lastError());
                    opMode.telemetry.addLine("Read this screen, then press STOP to exit autonomous.");
                    addDebugTelemetry();
                    opMode.telemetry.update();
                    opMode.idle();
                }
            }
        }catch(java.util.concurrent.CancellationException e){
            // STOP is a normal exit, not an autonomous fault.
        }catch(RuntimeException e){
            com.qualcomm.robotcore.util.RobotLog.ee("BioBuzzAuto",e,"Autonomous hardware/controller fault");
            opMode.telemetry.addData("Autonomous stopped",e.getMessage());opMode.telemetry.update();
        }finally{close();}
    }
    /** Also called when STOP is pressed during INIT before run() ever starts. */
    public void close(){
        if(closed)return;closed=true;
        SafeShutdown.run(routine::stop,drive::stop,shooter::stop,intake::stop,flowerIntake::lift,()->{if(camera!=null)camera.close();});
    }
}
