package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ebotsenums.StartingSide;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.ebotsutil.Pose;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.autonroutines.EbotsAutonRoutine;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.autonroutines.RoutineBlueCarousel;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.autonroutines.RoutineCarousel;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.autonroutines.RoutineWarehouse;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.autonroutines.RoutineWarehouseWithCollect;

public class StateConfigureRoutine implements EbotsAutonState{

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    StopWatch stopWatch;
    long touchSensorTimer;
    StartingSide startingSide;
    private Telemetry telemetry;
    private EbotsAutonOpMode autonOpMode;
    private DelayTime delayTime = DelayTime.ZERO;

    public enum DelayTime{
        ZERO,
        FIVE_SECONDS,
        TEN_SECONDS
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public StateConfigureRoutine(EbotsAutonOpMode autonOpMode){
        this.autonOpMode = autonOpMode;
        this.telemetry = autonOpMode.telemetry;
        HardwareMap hardwareMap = autonOpMode.hardwareMap;
        startingSide = autonOpMode.getStartingSide();

        stopWatch = new StopWatch();
        touchSensorTimer  = 500;
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    @Override
    public boolean shouldExit() {
        boolean shouldExit = false;
        if (autonOpMode.gamepad1.left_bumper && autonOpMode.gamepad1.right_bumper){
           shouldExit = true;
        }
        return autonOpMode.isStarted() | shouldExit | autonOpMode.isStopRequested();
    }
// if blue set heading to -heading
    @Override
    public void performStateActions() {
        boolean lockOutActive = stopWatch.getElapsedTimeMillis() <= touchSensorTimer;
        boolean allianceToggleRequested = autonOpMode.gamepad1.left_bumper &&
                autonOpMode.gamepad1.triangle;
        boolean startingSideToggleRequested = autonOpMode.gamepad1.left_bumper &&
                autonOpMode.gamepad1.cross;

        boolean moreDelayRequested = autonOpMode.gamepad1.left_bumper &&
                autonOpMode.gamepad1.dpad_right;

        boolean lessDelayRequested = autonOpMode.gamepad1.left_bumper &&
                autonOpMode.gamepad1.dpad_left;



        if (allianceToggleRequested && !lockOutActive) {
            stopWatch.reset();
            if (AllianceSingleton.getAlliance() == Alliance.BLUE) {
                AllianceSingleton.setAlliance(Alliance.RED);
            } else {
                AllianceSingleton.setAlliance(Alliance.BLUE);
            }
        } else if (startingSideToggleRequested && !lockOutActive) {
            stopWatch.reset();
            if (this.startingSide == StartingSide.CAROUSEL) {
                startingSide = StartingSide.WAREHOUSE;
            } else {
                startingSide = StartingSide.CAROUSEL;
            }
        } else if(moreDelayRequested && !lockOutActive){
            stopWatch.reset();
            if(delayTime == DelayTime.ZERO){
                delayTime = DelayTime.FIVE_SECONDS;
            } else if(delayTime == DelayTime.FIVE_SECONDS){
                delayTime = DelayTime.TEN_SECONDS;
            }
        }else if(lessDelayRequested && !lockOutActive){
            stopWatch.reset();
            if(delayTime == DelayTime.TEN_SECONDS) {
                delayTime = DelayTime.FIVE_SECONDS;
            } else if (delayTime == DelayTime.FIVE_SECONDS){
                delayTime = DelayTime.ZERO;
            }
        }
        telemetry.addData("alliance color: ", AllianceSingleton.getAlliance());
        telemetry.addData("starting side: ", startingSide);
        telemetry.addData("delay: ", delayTime.name());
    }

    @Override
    public void performTransitionalActions() {
        telemetry.addData("Transitioning out of", this.getClass().getSimpleName());
        telemetry.update();
        autonOpMode.setStartingSide(this.startingSide);
        double initialHeadingDeg = (AllianceSingleton.getAlliance() == Alliance.BLUE) ? -90 : 90;
        autonOpMode.initEbotsImu();
        autonOpMode.setInitialHeadingDeg(initialHeadingDeg);

        // Note that IMU and Alliance must be set prior to initializing pose
        Pose startingPose = new Pose(AllianceSingleton.getAlliance(), startingSide);
        autonOpMode.setCurrentPose(startingPose);

        if (delayTime == DelayTime.FIVE_SECONDS){
            // add delay to itinerary
            autonOpMode.appendStateToItinerary(StateDelayFiveSeconds.class);
        }else if (delayTime == DelayTime.TEN_SECONDS){
            // add delay to itinerary
            autonOpMode.appendStateToItinerary(StateDelayTenSeconds.class);
        }

        EbotsAutonRoutine routine;
        if (startingSide == StartingSide.CAROUSEL && !AllianceSingleton.isBlue()){
            routine = new RoutineCarousel();
        } else if(startingSide == StartingSide.CAROUSEL && AllianceSingleton.isBlue()){
            routine = new RoutineBlueCarousel();
        } else {
            routine = new RoutineWarehouseWithCollect();
        }
        autonOpMode.appendStatesToRoutineItinerary(routine);
        Log.d("EBOTS", "Completed StateConfigureRoutine");
    }
}
