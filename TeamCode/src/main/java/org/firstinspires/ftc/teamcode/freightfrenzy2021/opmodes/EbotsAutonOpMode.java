package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ebotsenums.BarCodePosition;
import org.firstinspires.ftc.teamcode.ebotsenums.StartingSide;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsWebcam;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.ebotsutil.Pose;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Bucket;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.AutonDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.AutonDriveVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.autonroutines.EbotsAutonRoutine;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.navigators.NavigatorVuforia;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.opencvpipelines.FreightDetector;

import java.util.ArrayList;

public abstract class EbotsAutonOpMode extends LinearOpMode {

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    protected BarCodePosition barCodePosition;

    // AllianceSingleton is managed so it can be passed to Teleop for FieldOrientedDrive
    protected EbotsWebcam frontWebcam;

    protected StartingSide startingSide = StartingSide.CAROUSEL;

    protected ArrayList<Class> itinerary = new ArrayList<>();

    // motion controller
    protected AutonDrive motionController;

    protected Pose currentPose;

    protected NavigatorVuforia navigatorVuforia;

    public Bucket bucket;
    private EbotsImu ebotsimu;
    public FreightDetector freightDetector;

    private int strafeClicksCollect = 0;
    private int forwardClicksCollect = 0;
    private int forwardClicksPushOff = 0;
    private int forwardClicksEnterWarehouse = 0;


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /**
     * Sets the AllianceSingleton to desired Alliance color
     * @param alliance enum value of alliance color
     */
    public void setAlliance(Alliance alliance){
        AllianceSingleton.setAlliance(alliance);
    }

    public Alliance getAlliance() {
        return AllianceSingleton.getAlliance();
    }

    public StartingSide getStartingSide() {
        return startingSide;
    }

    public double getCurrentHeadingDeg(boolean forceHardwareRead){
        ebotsimu = EbotsImu.getInstance(hardwareMap);
        return ebotsimu.getCurrentFieldHeadingDeg(forceHardwareRead);
    }

    public Pose getCurrentPose() {
        if (currentPose==null) currentPose = new Pose();    // null protect the getter
        return currentPose;
    }

    public AutonDrive getMotionController() {
        // because motors may be in the wrong mode, must init the motors when passing to a State
//        motionController.initMotorModes();
        return motionController;
    }

    public NavigatorVuforia getNavigatorVuforia() {
        return navigatorVuforia;
    }

    public BarCodePosition getBarCodePosition() {
        return barCodePosition;
    }

    public EbotsWebcam getFrontWebcam() {
        return frontWebcam;
    }

    public FreightDetector getFreightDetector() {
        return freightDetector;
    }

    public int getStrafeClicksCollect() {
        return strafeClicksCollect;
    }

    public void setStrafeClicksCollect(int strafeClicksCollect) {
        this.strafeClicksCollect = strafeClicksCollect;
    }

    public int getForwardClicksCollect() {
        return forwardClicksCollect;
    }

    public int getForwardClicksPushOff() {
        return forwardClicksPushOff;
    }

    public void setForwardClicksPushOff(int forwardClicksPushOff) {
        this.forwardClicksPushOff = forwardClicksPushOff;
    }

    public int getForwardClicksEnterWarehouse() {
        return forwardClicksEnterWarehouse;
    }

    public void setForwardClicksEnterWarehouse(int forwardClicksEnterWarehouse) {
        this.forwardClicksEnterWarehouse = forwardClicksEnterWarehouse;
    }

    public void setForwardClicksCollect(int forwardClicksCollect) {
        this.forwardClicksCollect = forwardClicksCollect;
    }

    public void setStartingSide(StartingSide startingSide){
        this.startingSide = startingSide;
    }

    public void setBarCodePosition(BarCodePosition barCodePosition) {
        this.barCodePosition = barCodePosition;
    }

    public void setInitialHeadingDeg(double initialHeadingDeg) {
        ebotsimu = EbotsImu.getInstance(hardwareMap);
        // initial heading is managed by the imu
        ebotsimu.setFieldHeadingWhenInitializedDeg(initialHeadingDeg);
    }

    public void setCurrentPose(Pose currentPose) {
        this.currentPose = currentPose;
    }

    public void setMotionController(AutonDrive motionController) {
        this.motionController = motionController;
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    public abstract void initAutonOpMode();

    public void appendStateToItinerary(Class autonStateClass){
        this.itinerary.add(autonStateClass);
    }

    public void appendStatesToRoutineItinerary(EbotsAutonRoutine routine){
        this.itinerary.addAll(routine.getRoutineItinerary());
    }

    public void clearRemainingItinerary(){
        itinerary.clear();
    }

    public void initEbotsImu(){
        ebotsimu = EbotsImu.getInstance(hardwareMap);
        ebotsimu.initEbotsImu(hardwareMap);
    }

}
