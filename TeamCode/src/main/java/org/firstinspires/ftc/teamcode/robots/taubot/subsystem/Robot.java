package org.firstinspires.ftc.teamcode.robots.taubot.subsystem;

import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.MAX_CHASSIS_LENGTH;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.MIN_SAFE_CHASSIS_LENGTH;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.craneIK;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.wrapAngleRad;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.taubot.Field;
import org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832;
import org.firstinspires.ftc.teamcode.robots.taubot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.taubot.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.robots.taubot.util.PositionCache;
import org.firstinspires.ftc.teamcode.robots.taubot.util.TauPosition;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * @author Mahesh Natamai
 */

@Config(value = "PPRobot")
public class Robot implements Subsystem {

    public DriveTrain driveTrain;
    public Turret turret;
    public Crane crane;
    public UnderArm underarm;
    public Subsystem[] subsystems;
    public Field field;

    public PositionCache positionCache;
    public TauPosition currentTauPos;
    public boolean updatePositionCache = false;

    private long[] subsystemUpdateTimes;
    private boolean autoDumpEnabled, doubleDuckEnabled;

    private TauPosition pos;

    private boolean rightConeStack;
    private boolean autonTurnDoneTelemetry, autonOnPoleTelemetry;

    private final List<LynxModule> hubs;

    private VoltageSensor batteryVoltageSensor;

    private Articulation articulation;
    private final Map<Articulation, StateMachine> articulationMap;

    private Bitmap craneBitmap;
    private Mat craneMat;
    public static int CB_WIDTH = 320;
    public static int CB_HEIGHT = 240;
    public static int numConesToCycle = 1;
    boolean updatePoseHappens = false;

    double current_dx = 0;
    double current_dy = 0;
    int transferStage = 0;

    public Robot(HardwareMap hardwareMap, boolean simulated) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // initializing subsystems
        driveTrain = new DriveTrain(hardwareMap, this, simulated);
        turret = new Turret(hardwareMap, this, simulated);
        underarm = new UnderArm(hardwareMap, this, simulated);
        crane = new Crane(hardwareMap, this, simulated);

        positionCache = new PositionCache( 5);

        subsystems = new Subsystem[] {driveTrain, turret, crane, underarm}; //{driveTrain, turret, crane};
        subsystemUpdateTimes = new long[subsystems.length];

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        articulation = Articulation.MANUAL;

        articulationMap = new HashMap<>();

        craneBitmap = Bitmap.createBitmap(CB_WIDTH, CB_HEIGHT, Bitmap.Config.RGB_565);
        craneMat = new Mat(CB_HEIGHT, CB_WIDTH, CvType.CV_8UC3);
        field = new Field(true);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("Articulation", articulation);
        telemetryMap.put("Unfolded", unfolded);
        telemetryMap.put("AutonState", autonIndex);
        telemetryMap.put("AutonTurnSt atus", autonTurnDoneTelemetry);
        telemetryMap.put("AutonOnPoleStatus", autonOnPoleTelemetry);
        telemetryMap.put("TransferState", transferStage);
        telemetryMap.put("Time Thing", timeSupervisor);
        telemetryMap.put("Auton Time", (totalAutonTime-System.nanoTime())/1e9);
        telemetryMap.put("Unfold Stage", unfoldStage);
        telemetryMap.put("auto-dump enabled", autoDumpEnabled);
        telemetryMap.put("Cancel Transfer Stage", cancelTransferIndex);

        for (int i = 0; i < subsystems.length; i++) {
            String name = subsystems[i].getClass().getSimpleName();
            telemetryMap.put(name + " Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (subsystemUpdateTimes[i] * 1e-6), (int) (1 / (subsystemUpdateTimes[i] * 1e-9))));
        }

        if(debug) {
            telemetryMap.put("Memory Pose X", pos.getPose().getX());
            telemetryMap.put("Memory Pose Y", pos.getPose().getY());
            telemetryMap.put("Memory Pose Heading", pos.getPose().getHeading());
            telemetryMap.put("Memory Turret Heading", pos.getTurretHeading());
        }
        telemetryMap.put("Update Pose happens " ,updatePoseHappens);

        Pose2d target =field.targetCoordinate;
        telemetryMap.put("Delta Time", deltaTime);
        telemetryMap.put("Target X", target.getX());
        telemetryMap.put("Target Y", target.getY());
        telemetryMap.put(" X", current_dx);
        telemetryMap.put(" Y", current_dy);
        telemetryMap.put("imu heading",driveTrain.getRawHeading());
        telemetryMap.put("turn until degrees done", turnUntilDegreesDone);
        telemetryMap.put("Scoring Pattern", field.getPatternIndex());
        telemetryMap.put("Pole Index", field.getScoringTargetIndex());
        telemetryMap.put("Pattern Name", field.getPatternName());


        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Robot";
    }


    public void fetchCachedTauPosition(){
        pos = positionCache.readPose();
    }
    public void resetRobotPosFromCache(Constants.Position start, double loggerTimeoutMinutes, boolean ignoreCache){
        fetchCachedTauPosition();
        //driveTrain.resetDrivetrainPos(start, pos, loggerTimeoutMinutes);
        //turret.resetTurretHeading(pos, loggerTimeoutMinutes);
        driveTrain.resetEncoders();
        if(PowerPlay_6832.gameState.equals(PowerPlay_6832.GameState.AUTONOMOUS)) {
            driveTrain.setPoseEstimate(new Pose2d(start.getPose().getX(), start.getPose().getY()));
            //articulate(Robot.Articulation.UNFOLD);
            articulate(Robot.Articulation.MANUAL);
            turret.articulate(Turret.Articulation.lockToZero);
            turret.setHeading((turret.TUNEABLE_SIZING_OFFSET/turret.TRANSFER_TICS)*Math.PI);
        }else if (PowerPlay_6832.gameState.equals(PowerPlay_6832.GameState.TEST) || PowerPlay_6832.gameState.equals(PowerPlay_6832.GameState.DEMO)){
            driveTrain.setPoseEstimate(new Pose2d(start.getPose().getX(), start.getPose().getY()));
            articulate(Robot.Articulation.UNFOLD_WITH_PLATE);
            turret.articulate(Turret.Articulation.lockToZero);
            turret.setHeading((turret.TUNEABLE_SIZING_OFFSET/turret.TRANSFER_TICS)*Math.PI);
        }else if(PowerPlay_6832.gameState.equals(PowerPlay_6832.GameState.TELE_OP)){
            int loggerTimeout = (int)(loggerTimeoutMinutes*60000);
            if(System.currentTimeMillis()-pos.getTimestamp()>loggerTimeout||ignoreCache) { //dont use cached position
                driveTrain.setPoseEstimate(new Pose2d(start.getPose().getX(), start.getPose().getY()));
                articulate(Robot.Articulation.MANUAL);
                //articulate(Robot.Articulation.UNFOLD);
                turret.articulate(Turret.Articulation.lockToZero);
                //turret.setHeading(0);
            }
            else { //apply cached position
                driveTrain.setPoseEstimate(new Pose2d(pos.getPose().getX(),pos.getPose().getY(),0));
                driveTrain.setHeading(pos.getPose().getHeading());
                driveTrain.setChassisLength(Constants.CONESTACK_CHASSIS_LENGTH);
                articulate(Articulation.MANUAL);
                turret.setHeading(pos.getTurretHeading());
                turret.setTicks(pos.getTurretTicks());
            }
            crane.tuckNudgeStick();
        }
    }

    public void clearBulkCaches(){
        for (LynxModule module : hubs)
            module.clearBulkCache();
    }

    public void start(){
        driveTrain.articulate(DriveTrain.Articulation.unlock);
        underarm.articulate(UnderArm.Articulation.manual);
        crane.enableAllPID();
        //turret.articulate(Turret.Articulation.runToAngle);
    }

    public double deltaTime = 0;
    long lastTime = 0;

    @Override
    public void update(Canvas fieldOverlay) {

        deltaTime = (System.nanoTime()-lastTime)/1e9;
        lastTime = System.nanoTime();

        clearBulkCaches(); //ALWAYS FIRST LINE IN UPDATE

        articulate(articulation);

        //update subsystems
        for(int i = 0; i < subsystems.length; i++) {
            Subsystem subsystem = subsystems[i];
            long updateStartTime = System.nanoTime();
            subsystem.update(fieldOverlay);
            subsystemUpdateTimes[i] = System.nanoTime() - updateStartTime;
        }
        if(updatePositionCache) {
            currentTauPos = new TauPosition(driveTrain.getPoseEstimate(), turret.getHeading(), turret.getTicks());
            positionCache.update(currentTauPos, false);
        }
        //paint the robot's current pose for dashboard
        //todo - update this Reach code to show the arm for the Tombot style crane
        double theta1 = wrapAngleRad(Math.toRadians(90 - crane.getShoulderAngle()));
        //double theta2 = -wrapAngleRad(Math.toRadians(180 - crane.getElbowTargetAngle()));
        //double height = SHOULDER_TO_ELBOW * Math.sin(theta1) + ELBOW_TO_WRIST * Math.sin(theta1 + theta2);

        //example of how Reach did autoDumping
        //if(autoDumpEnabled && height > HIGH_TIER_SHIPPING_HUB_HEIGHT - SHOULDER_AXLE_TO_GROUND_HEIGHT && crane.getBucketDistance() < org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem.Crane.BUCKET_TRIGGER_DISTANCE && articulation == org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem.Robot.Articulation.MANUAL && crane.getArticulation() == org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem.Crane.Articulation.MANUAL)
        //    articulation = org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem.Robot.Articulation.DUMP_AND_SET_CRANE_FOR_TRANSFER;

        //double wristAngle = wrapAngleRad(Math.toRadians(180) - wrapAngleRad(-(theta1 + theta2) + Math.toRadians(crane.getWristTargetAngle())));

        double x = (CB_WIDTH / 2.0);
        double y = (CB_HEIGHT / 2.0);
        double x1 = x + 70.1575 * Math.cos(theta1);
        double y1 = y + 70.1575 * Math.sin(theta1);
        //double x2 = x1 + 55.118 * Math.cos(theta1 + theta2);
        //double y2 = y1 + 55.118 * Math.sin(theta1 + theta2);


        craneMat.setTo(new Scalar(0));
        Imgproc.line(craneMat, new Point(x, CB_HEIGHT - y), new Point(x1, CB_HEIGHT - y1), new Scalar(255, 255, 255), 2);
        //Imgproc.line(craneMat, new Point(x1, CB_HEIGHT - y1), new Point(x2, CB_HEIGHT - y2), new Scalar(255, 255, 255), 2);
        //Imgproc.line(craneMat, new Point(x2, CB_HEIGHT - y2), new Point(x3, CB_HEIGHT - y3), new Scalar(255, 255, 255), 2);
        //Imgproc.line(craneMat, new Point(x2, CB_HEIGHT - y2), new Point(x4, CB_HEIGHT - y4), new Scalar(255, 255, 255), 2);

        Utils.matToBitmap(craneMat, craneBitmap);

        DashboardUtil.drawRobot(fieldOverlay, driveTrain.getPoseEstimate(), driveTrain.getWheelVelocities(), turret.getHeading(), crane.getShoulderAngle(), crane.getExtendInches(), crane.fieldPositionTarget);
    }

    public Bitmap getBitmap() {
        return craneBitmap;
    }

    @Override
    public void stop() {

        for(Subsystem subsystem: subsystems)
            subsystem.stop();

        currentTauPos = new TauPosition(driveTrain.getPoseEstimate(), turret.getHeading(), turret.getTicks());
        positionCache.update(currentTauPos, true);
    }

    public double getVoltage(){return batteryVoltageSensor.getVoltage();}

    int autonIndex = 0;

    boolean turnUntilDegreesDone = false;
    long autonTime;
    boolean initAuton = false;
    boolean autonRunWithinTime = true;
    long totalAutonTime;


    boolean turnDone = false;

    int timeSupervisor = 0;

    public static double mainTargetPoleX = 2*Field.INCHES_PER_GRID;
    public static double mainTargetPoleY = 0;

    public static double rightAltTargetPoleX = 2*Field.INCHES_PER_GRID;
    public static double rightAltTargetPoleY = -Field.INCHES_PER_GRID;

    public static double leftAltTargetPoleX = 2*Field.INCHES_PER_GRID;
    public static double leftAltTargetPoleY = Field.INCHES_PER_GRID;

    double targetPoleX;
    double targetPoleY;
    double targetPoleZ;

    public boolean AutonRun(int autonTarget, Constants.Position startingPosition, boolean targetAltCone){
        //TODO - make sure we aren't controlling conestack when supervisor takes over (multiple control risk)
        if(autonTarget == -1){
            autonTarget = 1;
        }
        if(startingPosition.equals(Constants.Position.START_LEFT))
            rightConeStack = false;
        else
            rightConeStack = true;


        switch (timeSupervisor) {
            case 0:
                //init stuff
                driveTrain.articulate(DriveTrain.Articulation.unlock);
                crane.setCraneTarget(driveTrain.getPoseEstimate().getX()+6,driveTrain.getPoseEstimate().getY(),8);
                totalAutonTime = futureTime(28);
                autonIndex = 0;

                if(targetAltCone){
                    if(startingPosition.equals(Constants.Position.START_LEFT))
                    {
                        targetPoleX = leftAltTargetPoleX;
                        targetPoleY = leftAltTargetPoleY;
                        targetPoleZ = 27;
                    }
                    else
                    {
                        targetPoleX = rightAltTargetPoleX;
                        targetPoleY = rightAltTargetPoleY;
                        targetPoleZ = 27;
                    }

                }
                else {
                    targetPoleX = mainTargetPoleX;
                    targetPoleY = mainTargetPoleY;
                    targetPoleZ = 37;
                }

                timeSupervisor++;
                break;
            case 1:
                //move on to the parking section of autonomous if enough time has passed
                //todo - 28 seconds is waiting too long and does not give time to park
                if(System.nanoTime() >= totalAutonTime){
                    timeSupervisor++;
                }
                switch (autonIndex) {
                    case 0:
                        driveTrain.setMaintainHeadingEnabled(false);
                        //drive away from the wall to give space to unfold
                        //driveTrain.articulate(DriveTrain.Articulation.unlock);
                        //turret.articulate(Turret.Articulation.lockToZero);
                        //crane.articulate(Crane.Articulation.manual);
                        //autonTime = futureTime(3);
                        if(unfold())
                        {
                            autonTime = futureTime(1.0);
                            autonIndex++;
                        }
                        break;
                    case 1:

                        if(driveTrain.turnUntilDegrees(0) && System.nanoTime() > autonTime){
                            autonIndex++;
                        }
                        break;
                    case 2:
                        //drive to general parking location
                        if(System.nanoTime() > autonTime) {
                            autonIsDriving();
                            turnDone = false;
                            autonTurnDoneTelemetry = false;
                            autonOnPoleTelemetry = false;
                            if (driveTrain.driveUntilDegrees( 45.5, 0, 20)) {
                                driveTrain.maxTuck();
                                autonIndex++;
//                                autonIndex = 0;
//                                return true;
                            }
                        }
                        break;
                    case 3:
                        //drop cone at nearest high pole
                        //debug this using telemetry - one isn't returning true - either turndone or onpole
                        //freezes it until timesupervisor takes over
                        autonNotDriving();
                        turret.articulate(Turret.Articulation.runToAngle);
                        crane.articulate(Crane.Articulation.manual);
                        if (startingPosition.equals(Constants.Position.START_LEFT)) {
                            if (!turnDone && driveTrain.turnUntilDegrees(90)) {
                                driveTrain.tuck();
                                turnDone = true;
                                autonTurnDoneTelemetry = true;

                            }
                            if(turnDone) {
                                if (driveTrain.driveUntilDegrees(-Field.INCHES_PER_GRID, 90, 20)) {
                                    autonIndex++;
                                }
                            }
                        }
                        else {
                            if (!turnDone && driveTrain.turnUntilDegrees(-90)) {
                                driveTrain.tuck();
                                turnDone = true;
                                autonTurnDoneTelemetry = true;

                            }
                            if(turnDone) {
                                if (driveTrain.driveUntilDegrees(-Field.INCHES_PER_GRID, -90, 20)) {
                                    autonIndex++;
                                }
                            }

                        }
                        autonTime = futureTime(.3);
                        break;
                    case 4:
                        if(System.nanoTime() > autonTime)
                            autonIndex ++;
                        break;
                    case 5:
                        //targets middle pole
                        driveTrain.setMaintainHeadingEnabled(true);
                        if(startingPosition.equals(Constants.Position.START_LEFT)) {
                            if (crane.goToFieldCoordinate(targetPoleX-7, targetPoleY-7, targetPoleZ-3)) {
                                autonTime = futureTime(0.8);
                                autonIndex++;
                            }
                        }else{
                            if (crane.goToFieldCoordinate(targetPoleX-7, targetPoleY+7, targetPoleZ-3)) {
                                autonTime = futureTime(0.8);
                                autonIndex++;
                            }
                        }
                        break;
                    case 6:
                        if(startingPosition.equals(Constants.Position.START_LEFT)){
                            if (crane.goToFieldCoordinate(targetPoleX-7, targetPoleY-7, targetPoleZ-3)) {
                                autonTime = futureTime(0.8);
                                autonIndex++;
                            }
                        }else{
                            if (crane.goToFieldCoordinate(targetPoleX-7, targetPoleY+7, targetPoleZ-3)) {
                                autonTime = futureTime(0.8);
                                autonIndex++;
                            }
                        }
                        if (System.nanoTime() >= autonTime) {
                            crane.setGripper(false);
                            autonTime = futureTime(0.3);
                            autonIndex++;
                        }
                        break;
                    case 7:
                        if(System.nanoTime() >= autonTime) {
                            if(crane.goHome()) {
                                autonIndex++;
                            }
                        }
                        break;
                    case 8:
                        if (System.nanoTime() >= autonTime) {
                            autonIndex++;
                            //runs cone stack articulation but no longer works with underarm in the way
                            //if we r on left side run cone stack left, if right run right cone stack
                            /*
                            if (startingPosition.equals(Constants.Position.START_LEFT)) {
                                crane.articulate(Crane.Articulation.coneStackLeft);
                            } else {
                                crane.articulate(Crane.Articulation.coneStackRight);
                            }
                            if (crane.getArticulation() == Crane.Articulation.manual) {
                                autonIndex++;
                            }

                             */
                        }
                        break;
                    case 9:
                        autonIndex = 0;
                        //TODO - REMOVE RETURN
//                        if(true) return true;
                        timeSupervisor++;
                        break;
                }
                break;
            case 2:
                autonIndex = 0;
                driveTrain.setMaintainHeadingEnabled(false);
                crane.articulate(Crane.Articulation.home);
                driveTrain.setChassisLength(MIN_SAFE_CHASSIS_LENGTH);
                autonIsDriving();

                if (crane.getArticulation() == Crane.Articulation.manualDrive) {
                    crane.articulate(Crane.Articulation.manual);
                }

                //middle parking position/no parking target found
                if (autonTarget == 1) {
                    if(startingPosition.equals(Constants.Position.START_LEFT)){
                        if (driveTrain.driveUntilDegrees(1*Field.INCHES_PER_GRID, 90, 20))
                            timeSupervisor++;
                    }else{
                        if (driveTrain.driveUntilDegrees(1*Field.INCHES_PER_GRID, -90, 20))
                            timeSupervisor++;
                    }
                }

                //go to other parking position based on visual target
                if (startingPosition.equals(Constants.Position.START_LEFT)) {
                    if (autonTarget == 0) {
                        if (driveTrain.driveUntilDegrees(2 * Field.INCHES_PER_GRID-6, 90, 20)) {
                            timeSupervisor++;
                        }
                    } else if (autonTarget == 2 || Objects.isNull(autonTarget)){
                        driveTrain.maxTuck();
                        timeSupervisor++;
                    }
                } else {
                    if (autonTarget == 2) {
                        if (driveTrain.driveUntilDegrees(2 * Field.INCHES_PER_GRID-6, -90, 20)) {
                            timeSupervisor++;
                        }
                    } else if (autonTarget == 0 || Objects.isNull(autonTarget)) {
                        driveTrain.maxTuck();
                        timeSupervisor++;
                    }
                }
                break;
            case 3:
                if(startingPosition.equals(Constants.Position.START_LEFT)){
                    if(autonTarget != 0){
                        if(driveTrain.turnUntilDegrees(180)){
                            timeSupervisor++;
                        }
                    }else{
                        timeSupervisor++;
                    }
                }else{
                    if(autonTarget != 2){
                        if(driveTrain.turnUntilDegrees(180)){
                            timeSupervisor++;
                        }
                    }else{
                        timeSupervisor++;
                    }
                }
                break;
            case 4:
                if(driveTrain.driveUntilDegrees(0.3*Field.INCHES_PER_GRID,180,20)){
                    autonNotDriving();
                    crane.articulate(Crane.Articulation.manual);
                    timeSupervisor++;
                }
                break;
            case 5:
                if(crane.unfoldTransferPlate()){
                    crane.articulate(Crane.Articulation.lockToHome);
                    timeSupervisor++;
                }
                break;
            case 6:
                driveTrain.maxTuck();
                timeSupervisor = 0;
                return true;
        }
        return false;
    }


    public void autonIsDriving(){
        crane.articulate(Crane.Articulation.autonDrive);
    }

    public void autonNotDriving(){
        crane.articulate(Crane.Articulation.manual);
    }

    //----------------------------------------------------------------------------------------------
    // Articulations
    //----------------------------------------------------------------------------------------------

    public enum Articulation {
        MANUAL,
        AUTON,
        ROBOTDRIVE,
        UNFOLD,
        CALIBRATE,
        UNDERARM_CONESTACK,

        // misc. articulations
        INIT,
        START,
        CANCEL_TRANSFER,
        START_DOWN, // use to prep for start - stows the crane
        START_END_GAME, //use on a timer to automatically deploy carousel spinner 10 seconds before end game

        // tele-op articulations
        TRANSFER,
        DROP,
        UNFOLD_WITH_PLATE

    }

    boolean unfolded = false;
    boolean driveInit = false;
    boolean notDriveInit = false;

    public Articulation articulate(Articulation target) {
        articulation = target;
        if(isDriverDriving()) { //bypass the normal articulation flow when driving - this could short circuit other articulations leaving them in unknown stages
            resetArticulations();
            driveTrain.articulate(DriveTrain.Articulation.unlock);
            underarm.articulate(UnderArm.Articulation.driving);
            if(!driveInit){
                crane.articulate(Crane.Articulation.lockToTransfer); //keeps crane in safe position
                driveInit = true;
            }
            notDriveInit = false;
        }
        else{
            driveInit = false;
            if(!notDriveInit){
                driveTrain.articulate(DriveTrain.Articulation.lockWheels);
                underarm.articulate(UnderArm.Articulation.manual);
                notDriveInit = true;
            }
            switch (this.articulation) {
                case MANUAL:

                    break;
                case CANCEL_TRANSFER:
                    if(cancelTransfer()) {
                        articulation = Articulation.MANUAL;
                    }
                case UNFOLD_WITH_PLATE:
                    if(unfoldWithPlate()){
                        unfolded = true;
                        articulation = Articulation.MANUAL;
                    }
                    break;
                case CALIBRATE:
                    if (crane.calibrate()) {
                        articulation = Articulation.INIT;
                    }
                    break;
                case TRANSFER:
                    if (transfer()) {
                        turret.articulate(Turret.Articulation.runToAngle);
                        crane.articulate(Crane.Articulation.scoreCone);
                        //underarm.articulate(UnderArm.Articulation.substationHover);
                        articulation = Articulation.MANUAL;
                    }
                    break;
                case UNDERARM_CONESTACK:
                    if(underarmConeStack()) {
                        articulation = Articulation.MANUAL;
                    }
                    break;
                case DROP:
                    if (drop()) {
                        articulation = Articulation.MANUAL;
                    }
                    break;
                case UNFOLD:
                    if (unfold()) {
                        unfolded = true;
                        articulation = Articulation.MANUAL;
                    }
                    break;
                case INIT:
                    crane.articulate(Crane.Articulation.init);
                    underarm.articulate(UnderArm.Articulation.fold);
                    break;
            }
        }
        return this.articulation;
    }

    int plateUnfoldStage = 0;
    public boolean unfoldWithPlate(){
        switch (plateUnfoldStage){
            case 0:
                if(unfold()){
                    plateUnfoldStage++;
                    crane.articulate(Crane.Articulation.manual);
                }
                break;
            case 1:
                if(crane.unfoldTransferPlate()){
                    plateUnfoldStage++;
                }
                break;
            case 2:
                plateUnfoldStage = 0;
                return true;
        }
        return false;
    }
    public boolean getAutonConeStack() {
        return rightConeStack;
    }
    public Articulation getArticulation(){
        return articulation;
    }


    public boolean isDriverDriving() {
        return driverDriving;
    }

    long driveAllowedSwitchTimer = 0;
    public void setDriverDriving(boolean driverDriving) {
        if(driverDriving == true){
            driveAllowedSwitchTimer = futureTime(0.3);
            this.driverDriving = true;
        }
        if(System.nanoTime() > driveAllowedSwitchTimer || driverDriving == false){
            this.driverDriving = false;
        }
    }

    private boolean driverDriving = false;

    public void driverIsNowDriving(){
        setDriverDriving(true);
    }

    public void driverHasStoppedDriving(){
        setDriverDriving(false);
    }

    public boolean checkCollision(double calcShoulder, double calcTurret){
        if(Math.abs(calcTurret - Math.toDegrees(driveTrain.getRawHeading())) < 20) {
            double maxHeight = Math.max(UnderArm.UPPER_ARM_LENGTH * Math.sin(Math.toRadians(underarm.getCalculatedShoulderAngle())), underarm.getCalculatedHeight()) - (Crane.shoulderHeight*Constants.INCHES_PER_METER - UnderArm.shoulderHeight);
            double x = (maxHeight / Math.tan(Math.toRadians(calcShoulder))) - driveTrain.getChassisLength() - Turret.turretOffset;

            if (x > 0) {
                return true;
            }

        }

        return false;
    }

    int unfoldStage = 0;
    long unfoldTimer = 0;

    public boolean  unfold(){
        switch (unfoldStage){
            case 0:
                driveTrain.articulate(DriveTrain.Articulation.unlock);
                crane.articulate(Crane.Articulation.manual);
                underarm.articulate(UnderArm.Articulation.manual);
                crane.setShoulderTargetAngle(30);
                crane.setExtendTargetPos(.3);
                driveTrain.setChassisLength(MAX_CHASSIS_LENGTH/2);
                unfoldTimer = futureTime(.5);
                unfoldStage++;
                break;
            case 1:
                //drive away from wall to give flipper space to flip without extending over wall
                if(System.nanoTime() > unfoldTimer) {
                    //crane.setShoulderTargetAngle(60);
                    underarm.articulate(UnderArm.Articulation.unfold);
                    unfoldStage++;
                    unfoldTimer = futureTime(0.5);
                }
                break;
            case 2:
                if(System.nanoTime() > unfoldTimer) {
                    driveTrain.tuck();
                    unfoldStage++;
                }
                //if(driveTrain.driveUntilDegrees(32, 0, 20)) unfoldStage++;
                /*
                    crane.setShoulderTargetAngle(60);
                    underarm.articulate(UnderArm.Articulation.unfold);
                    unfoldTimer = futureTime(.5);
                    unfoldStage++;
                }*/

                break;
            case 3:
                if(System.nanoTime() > unfoldTimer){
                    //crane.flipToFlip();
                    unfoldTimer = futureTime(.3);
                    unfoldStage++;
                }
                break;
            case 4:
                if(System.nanoTime() > unfoldTimer){
                    //crane.flipToHome();
                    unfoldStage++;
                }
                break;
            case 5:
                //crane.articulate(Crane.Articulation.home);
                underarm.articulate(UnderArm.Articulation.unfold);
                crane.tuckNudgeStick(); //changed from extended to tucked (redundant, should already be tucked) - keep nudgestick tucked always, except for when approaching a pole
                unfoldStage = 0;
                return true;
        }
        return false;
    }

    int dropStage = 0;
    long dropTimer = 0;

    public boolean drop(){
        switch (dropStage){
            case 0:
                field.incTarget();
                crane.updateScoringPattern();
                dropStage++;
                break;
            case 1:
                crane.articulate(Crane.Articulation.dropConeReturnToTransfer);
                dropStage++;
                break;
            case 2:
                dropStage = 0;
                return true;
        }
        return false;
    }
    public void resetArticulations(){
        transferStage=0;
        dropStage=0;
        unfoldStage = 0;
        coneStackStage = 0;
        dropStage = 0;
        cancelTransferIndex = 0;
        underarm.resetArticulations();
        crane.resetArticulations();
    }
    public void underarmConeStackAdvance() {
        coneStackStage ++;
    }

    int coneStackStage = 0;
    long coneStackTimer = 0;
    public boolean underarmConeStack () {
        //from after the robot drives forward in auton to the end of cycling cones from conestack
        //will still have to handle park from within runAuton
//        ConeStack obj = field.getConeStack(rightConeStack);
//        Pose2d pos = Field.convertToInches(obj.getPosition());
        if(numConesToCycle > 0) {
            switch (coneStackStage) {
                case 0:
                    //so that underarm is facing conestack (don't know the actual angle)
//                    if(rightConeStack)
//                        if (driveTrain.turnUntilDegrees(90))
//                            coneStackStage++;
//                    if(!rightConeStack)
//                        if(driveTrain.turnUntilDegrees(-90));
                    driveTrain.setChassisLength(Constants.CONESTACK_CHASSIS_LENGTH);
                    underarm.release();
                    //coneStackStage++;
                    break;
                case 1:
                    //turn on vision pipeline and set chassis length and heading accordingly
//                    if (/* SOME KIND OF CHECK, PLEASE NO TIMER */ false

                    //coneStackStage++;
                    break;
                case 2:
                    if (underarm.atConeStack()) {
                        //coneStackStage++;
                    }
                    break;
                case 3:
                    underarm.grip();
                    if (/* SOME KIND OF CHECK, PLEASE NO TIMER */ false) {
                        //coneStackStage++;
                    }
                    break;
                case 4:
                    if(transfer()) {
                        //coneStackStage++;
                    }
//                    if (/* SOME KIND OF CHECK, PLEASE NO TIMER */ false)
//                        coneStackStage++;
                    break;
                case 5:
                    underarm.updateConeStackAngles();
                    field.getConeStack(rightConeStack).takeCone();
                    if (crane.goToFieldthing(field.objects[34])) {
                        //coneStackStage++;
                    }
                    break;
                case 6:
                    crane.release();
                    //coneStackStage++;
                    break;
                case 7:
                    crane.articulate(Crane.Articulation.transfer);
                    numConesToCycle --;
                    coneStackStage = 0;

            }
        }
        else
            return true;

        return false;

    }

    long transferTimer = 0;

    public void transferAdvance() {
        transferStage ++;
    }

    public boolean transfer(){

        switch (transferStage) {
            case 0: //move Crane to transfer position
                driveTrain.articulate(DriveTrain.Articulation.lockWheels);
                driveTrain.setChassisLength(Constants.MAX_CHASSIS_LENGTH);
                crane.release();
                crane.articulate(Crane.Articulation.transfer); //tells crane to go to transfer position
                transferTimer = futureTime(0.5);
                underarm.speedMode();
                transferStage++;
                break;
            case 1:
                if(System.nanoTime() > transferTimer && crane.atTransfer()){
                    underarm.articulate(UnderArm.Articulation.transfer); //tell underarm to place cone in holder
                    transferTimer = futureTime(1.0);
                    transferStage++;
                }
                break;
            case 2:
                if(System.nanoTime() >= transferTimer && underarm.atTransfer) {
                    crane.articulate(Crane.Articulation.postTransfer); //tell crane to pickup cone
                    transferTimer = futureTime(0.3);
                    transferStage++;
                }
                break;
            case 3:
                if(System.nanoTime() >= transferTimer && crane.atPostTransfer()) {
                    driveTrain.setChassisLength(Constants.MAX_CHASSIS_LENGTH); //gets out of way of holder
                    transferStage++;
                }
                break;
            case 4:
                transferStage++;
                transferTimer = futureTime(0.3);
                break;
            case 5:
                if(System.nanoTime() >= transferTimer) {
                    //driveTrain.setChassisLength(Constants.MAX_CHASSIS_LENGTH);
                    crane.articulate(Crane.Articulation.manual);
                    turret.articulate(Turret.Articulation.lockTo180);
                    underarm.regularMode();
                    transferStage++;
                }
                break;
            case 6:
                transferStage = 0;
                return true;
        }
        return false;
    }

    long cancelTransferTimer;
    int cancelTransferIndex = 0;
    public boolean cancelTransfer() {
        switch (cancelTransferIndex) {
            case (0):
            {
                underarm.setEnableLasso(false);
                transferStage = 0;
                turret.articulate(Turret.Articulation.transfer);
                crane.articulate(Crane.Articulation.manual);
                driveTrain.articulate(DriveTrain.Articulation.unlock);
                crane.setShoulderTargetAngle(Crane.SAFE_SHOULDER_ANGLE);
                cancelTransferTimer = futureTime(1);
                cancelTransferIndex++;
                break;
            }
            case (1):
            {
                if(System.nanoTime() > cancelTransferTimer) {
                    crane.articulate(Crane.Articulation.home);
                    turret.articulate(Turret.Articulation.runToAngle);
                    underarm.articulate(UnderArm.Articulation.home);
                    cancelTransferTimer = futureTime(0.7);
                    cancelTransferIndex++;
                }
                break;
            }
            case 2:
                underarm.setEnableLasso(true);
                cancelTransferIndex = 0;
                return true;


        }
        return false;
    }

    public void updateFieldTargetPose(double dx, double dy){
        updatePoseHappens = true;
        current_dx = dx;
        current_dy = dy;
        field.updateTargetPose(dx, dy, driveTrain);
    }
}
