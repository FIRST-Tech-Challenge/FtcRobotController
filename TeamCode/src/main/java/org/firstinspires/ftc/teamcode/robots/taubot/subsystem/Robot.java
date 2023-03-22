package org.firstinspires.ftc.teamcode.robots.taubot.subsystem;

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
import org.firstinspires.ftc.teamcode.robots.taubot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.taubot.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.Vector3;
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

    private long[] subsystemUpdateTimes;
    private boolean autoDumpEnabled, doubleDuckEnabled;

    private final List<LynxModule> hubs;

    private VoltageSensor batteryVoltageSensor;

    private Articulation articulation;
    private final Map<Articulation, StateMachine> articulationMap;

    private Bitmap craneBitmap;
    private Mat craneMat;
    public static int CB_WIDTH = 320;
    public static int CB_HEIGHT = 240;
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
        telemetryMap.put("TransferState", transferStage);
        telemetryMap.put("Time Thing", timeSupervisor);
        telemetryMap.put("Auton Time", (totalAutonTime-System.nanoTime())/1e9);
        telemetryMap.put("Unfold Stage", unfoldStage);
        telemetryMap.put("auto-dump enabled", autoDumpEnabled);

        for (int i = 0; i < subsystems.length; i++) {
            String name = subsystems[i].getClass().getSimpleName();
            telemetryMap.put(name + " Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (subsystemUpdateTimes[i] * 1e-6), (int) (1 / (subsystemUpdateTimes[i] * 1e-9))));
        }

        if(debug) {

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

    public void clearBulkCaches(){
        for (LynxModule module : hubs)
            module.clearBulkCache();
    }

    public void start(){
        driveTrain.articulate(DriveTrain.Articulation.unlock);
        crane.enableAllPID();
        turret.articulate(Turret.Articulation.runToAngle);
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

        DashboardUtil.drawRobot(fieldOverlay, driveTrain.getPoseEstimate(), driveTrain.getWheelVelocities(), turret.getTargetHeading(), crane.getShoulderAngle(), crane.getExtendInches());
    }

    public Bitmap getBitmap() {
        return craneBitmap;
    }

    @Override
    public void stop() {
        for(Subsystem subsystem: subsystems)
            subsystem.stop();
    }

    public double getVoltage(){return batteryVoltageSensor.getVoltage();}

    int autonIndex = 0;

    boolean turnUntilDegreesDone = false;
    long autonTime;
    boolean initAuton = false;
    boolean autonRunWithinTime = true;
    long totalAutonTime;


    boolean turnDone = false;
    boolean onPole = false;

    int timeSupervisor = 0;

    public boolean AutonRun(int autonTarget, Constants.Position startingPosition){

        switch (timeSupervisor) {
            case 0:
                driveTrain.articulate(DriveTrain.Articulation.unlock);
                crane.setCraneTarget(driveTrain.getPoseEstimate().getX()+6,driveTrain.getPoseEstimate().getY(),8);
                totalAutonTime = futureTime(28);
                autonIndex = 0;
                timeSupervisor++;
                break;
            case 1:
                if(System.nanoTime() >= totalAutonTime){
                    timeSupervisor++;
                }
                switch (autonIndex) {
                    case 0:
                        driveTrain.articulate(DriveTrain.Articulation.unlock);
                        turret.articulate(Turret.Articulation.lockToZero);
                        autonTime = futureTime(3);
                        autonIndex++;
                        break;
                    case 1:
                        //drive to general parking location
                        if(System.nanoTime() > autonTime) {
                            crane.articulate(Crane.Articulation.manual);
                            driverIsDriving();

                            turnDone = false;
                            onPole = false;
                            if (driveTrain.driveUntilDegrees(2 * Field.INCHES_PER_GRID - 4, 0, 20)) {
                                driveTrain.tuck();
                                autonIndex++;
                            }
                        }
                        break;
                    case 2:
                        //drop cone at nearest high pole
                        driverNotDriving();
                        turret.articulate(Turret.Articulation.runToAngle);
                        if (startingPosition.equals(Constants.Position.START_LEFT)) {
                            if (!turnDone && driveTrain.turnUntilDegrees(90)) {
                                turnDone = true;
                            }
                            if(!onPole && crane.goToFieldCoordinate(3 * Field.INCHES_PER_GRID - 2 , Field.INCHES_PER_GRID - 2.5, 39)){
                                onPole = true;
                            }
                            if(turnDone && onPole){
                                autonTime = futureTime(0.8);
                                autonIndex++;
                            }
                        }
                        else {
                            if (!turnDone && driveTrain.turnUntilDegrees(-90)) {
                                turnDone = true;
                            }
                            if(!onPole && crane.goToFieldCoordinate(3 * Field.INCHES_PER_GRID + 1, -Field.INCHES_PER_GRID - 1.2, 39)){
                                onPole = true;
                            }
                            if(turnDone && onPole){
                                autonTime = futureTime(0.8);
                                autonIndex++;
                            }
                        }
                        break;
                    case 3:
                        if(startingPosition.equals(Constants.Position.START_LEFT)){
                            crane.goToFieldCoordinate(3 * Field.INCHES_PER_GRID - 2 , Field.INCHES_PER_GRID - 2.5, 39);
                        }else{
                            crane.goToFieldCoordinate(3 * Field.INCHES_PER_GRID + 2, -Field.INCHES_PER_GRID - 1.0, 39);
                        }
                        if (System.nanoTime() >= autonTime) {
                            crane.setGripper(false);
                            autonTime = futureTime(0.3);
                            autonIndex++;
                        }
                        break;
                    case 4:
                        if(System.nanoTime() >= autonTime && crane.goHome()) {
                            autonIndex++;
                        }
                        break;
                    case 5:
                        if (System.nanoTime() >= autonTime) {
                            //if we r on left side run cone stack left, if right run right cone stack
                            if (startingPosition.equals(Constants.Position.START_LEFT)) {
                                crane.articulate(Crane.Articulation.coneStackLeft);
                            } else {
                                crane.articulate(Crane.Articulation.coneStackRight);
                            }
                            if (crane.getArticulation() == Crane.Articulation.manual) {
                                autonIndex++;
                            }
                        }
                        break;
                    case 6:
                        autonIndex = 0;
                        timeSupervisor++;
                        break;
                }
                break;
            case 2:
                autonIndex = 0;
                crane.articulate(Crane.Articulation.home);
                if (crane.getArticulation() == Crane.Articulation.manualDrive) {
                    crane.articulate(Crane.Articulation.manual);
                }
                if (autonTarget == 1 || Objects.isNull(autonTarget)) {
                    if(startingPosition.equals(Constants.Position.START_LEFT)){
                        if (driveTrain.driveUntilDegrees(1.5*Field.INCHES_PER_GRID-driveTrain.getPoseEstimate().getY(), 90, 20))
                            timeSupervisor++;
                    }else{
                        if (driveTrain.driveUntilDegrees(driveTrain.getPoseEstimate().getY()+1.5*Field.INCHES_PER_GRID, 270, 20))
                            timeSupervisor++;
                    }
                }
                if (startingPosition.equals(Constants.Position.START_LEFT)) {
                    if (autonTarget == 0) {
                        if (driveTrain.driveUntilDegrees(0.8 * Field.INCHES_PER_GRID-4, 90, 20))
                            timeSupervisor++;
                    } else if (autonTarget == 2) {
                        if (driveTrain.driveUntilDegrees(-0.8 * Field.INCHES_PER_GRID+4, 90, 20))
                            timeSupervisor++;
                    }
                } else {
                    if (autonTarget == 0) {
                        if (driveTrain.driveUntilDegrees(-0.8 * Field.INCHES_PER_GRID, 270, 20))
                            timeSupervisor++;
                    } else if (autonTarget == 2) {
                        if (driveTrain.driveUntilDegrees(0.8 * Field.INCHES_PER_GRID-3, 270, 20))
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
                    }
                }else{
                    if(autonTarget != 2){
                        if(driveTrain.turnUntilDegrees(180)){
                            timeSupervisor++;
                        }
                    }
                }
                break;
            case 4:
                crane.nudgeLeft();
                crane.setCraneTarget(turret.getTurretPosition().getX() - 2, turret.getTurretPosition().getY()-2, 26);
                crane.articulate(Crane.Articulation.manual);
                driveTrain.maxTuck();
                timeSupervisor = 0;
                return true;
        }
        return false;
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

        // misc. articulations
        INIT,
        START,
        START_DOWN, // use to prep for start - stows the crane
        START_END_GAME, //use on a timer to automatically deploy carousel spinner 10 seconds before end game

        // tele-op articulations
        TRANSFER,
        DROP

    }

    boolean unfolded = false;

    public Articulation articulate(Articulation target) {
        articulation = target;
        if(isDriverDriving()) { //bypass the normal articulation flow when driving - this could short circuit other articulations leaving them in unknown stages
            driveTrain.articulate(DriveTrain.Articulation.unlock);
            crane.articulate(Crane.Articulation.robotDriving); //keeps crane in safe position
            underarm.articulate(UnderArm.Articulation.home);
        }
        else{
            if(crane.getArticulation().equals(Crane.Articulation.robotDriving)){
                crane.articulate(Crane.Articulation.manualDrive);
            }
            switch (this.articulation) {
                case MANUAL:

                    break;
                case CALIBRATE:
                    if (crane.calibrate()) {
                        articulation = Articulation.INIT;
                    }
                    break;
                case TRANSFER:
                    if (transfer()) {
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

    public boolean isDriverDriving() {
        return driverDriving;
    }

    public void setDriverDriving(boolean driverDriving) {
        this.driverDriving = driverDriving;
    }

    private boolean driverDriving = false;

    public void driverIsDriving(){
        setDriverDriving(true);
    }

    public void driverNotDriving(){
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

    public boolean unfold(){
        switch (unfoldStage){
            case 0:
                driveTrain.articulate(DriveTrain.Articulation.unlock);
                crane.articulate(Crane.Articulation.manual);
                crane.setShoulderTargetAngle(70);
                unfoldTimer = futureTime(0.7);
                unfoldStage++;
                break;
            case 1:
                if(System.nanoTime() > unfoldTimer) {
                    underarm.articulate(UnderArm.Articulation.foldTransfer);
                    unfoldTimer = futureTime(1);
                    unfoldStage++;
                }
                break;
            case 2:
                if(System.nanoTime() > unfoldTimer){
                    unfoldStage++;
                }
                break;
            case 3:
                crane.articulate(Crane.Articulation.manualDrive);
                underarm.articulate(UnderArm.Articulation.home);
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
                crane.articulate(Crane.Articulation.dropCone);
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
    }

    long transferTimer = 0;

    public void transferAdvance() {
        transferStage ++;
    }
    public boolean transfer(){

        switch (transferStage) {
            case 0: //move Crane to transfer position
                //underarm.articulate((UnderArm.Articulation.manual));
                driveTrain.setChassisLength(Constants.MAX_CHASSIS_LENGTH);
                crane.release();
                crane.articulate(Crane.Articulation.transfer); //tells crane to go to transfer position
                transferStage++;
                transferTimer = futureTime(3.0);
                break;
            case 1:
                //if(crane.atTransferPosition() ){ //todo debug - temp switched to a timer because crane.atTransferPosition() not working
                if(System.nanoTime() >= transferTimer){
                    driveTrain.articulate(DriveTrain.Articulation.lock);
                    underarm.articulate(UnderArm.Articulation.transfer); //tell underarm to go to transfer angle
                    transferTimer = futureTime(1.0);
                    transferStage++;
                }
                break;
            case 2: //this is where we grab the cone with the bulb gripper
                //if(crane.atTransferPosition()&& underarm.atTransfer()){ //todo debug - temp switched to a timer because crane.atTransferPosition() not working
                if(System.nanoTime() >= transferTimer){
                    crane.grab();
                    transferTimer = futureTime(1.5);
                    //TODO TRIED VARIOUS THINGS BELOW TO ABORT/STALL SO WE COULD LOOK AT FINAL TRANSFER POSITION BUT THEY HAVE DIFFERENT SIDE EFFECTS FROM THE COMBINATIONS OF ARTICULATIONS
                    //CAN TEMPORARILY EXIT HERE FOR DEBUGGING
                    //boolean tuning = true; if (tuning) {transferStage = 0; return true;}
                    //todo TEMPORARILY STALLING HERE SO WE CAN SEE THE TRANSFER POSITION, UNCOMMENT NEXT LINE WHEN TUNED
                    transferStage++;
                }
                break;
            case 3:  //here we release the underarm's gripper
                if(System.nanoTime() >= transferTimer) {
                    underarm.release();
                    transferTimer = futureTime(0.3);
                    transferStage++;
                }
                break;
            case 4:
                if(System.nanoTime() >= transferTimer) {
                    driveTrain.articulate(DriveTrain.Articulation.unlock);
                    driveTrain.setChassisLength(Constants.MAX_CHASSIS_LENGTH - 5);
                    crane.articulate(Crane.Articulation.transferAdjust);
                    transferTimer = futureTime(1.0);
                    transferStage++;
                }

                break;
            case 5:
                if(System.nanoTime() >= transferTimer) {
                    underarm.grip(); //need to retract the gripper for pass through
                    underarm.articulate(UnderArm.Articulation.transferRecover);
                    //TODO UNCOMMENT NEXT LINE WHEN YOU WANT THE CRANE TO ACTUALLY ATTEMPT THE CONE DROP - BE SURE FIELD POSITIONING IS RIGHT
                    transferStage++;
                    transferTimer = futureTime(1.0);
                }
                break;
            case 6:
                if(System.nanoTime() >= transferTimer) {
                    driveTrain.setChassisLength(Constants.MAX_CHASSIS_LENGTH);
                    crane.articulate(Crane.Articulation.home); //allows crane to do crane things after transfer
                    transferStage = 0;
                    return true;
                }
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
