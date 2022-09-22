package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsBlinkin;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateCalibratingImu implements EbotsAutonState{
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private DigitalChannel frontRollerTouch;
    private DigitalChannel backRollerTouch;
    private boolean initComplete = false;
    private EbotsAutonOpMode autonOpMode;
    private HardwareMap hardwareMap;
    private EbotsImu ebotsImu;
    private EbotsBlinkin ebotsBlinkin;
    private boolean userRequestExit = false;
    private StopWatch stopWatchCalibration = new StopWatch();
    private boolean wasAligned = false;
    private long alignDuration = 750L;
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public StateCalibratingImu(EbotsAutonOpMode autonOpMode){
        this.hardwareMap = autonOpMode.hardwareMap;
        this.autonOpMode = autonOpMode;
        ebotsBlinkin = EbotsBlinkin.getInstance(hardwareMap);
        ebotsImu = EbotsImu.getInstance(hardwareMap);
        if (AllianceSingleton.isBlue()){
            frontRollerTouch = autonOpMode.hardwareMap.get(DigitalChannel.class, "leftFrontTouch");
            backRollerTouch = autonOpMode.hardwareMap.get(DigitalChannel.class, "leftBackTouch");
        } else {
            frontRollerTouch = autonOpMode.hardwareMap.get(DigitalChannel.class, "rightFrontTouch");
            backRollerTouch = autonOpMode.hardwareMap.get(DigitalChannel.class, "rightBackTouch");
        }

        ebotsBlinkin.lightsRed();
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
        userRequestExit = autonOpMode.gamepad1.left_bumper && autonOpMode.gamepad1.right_bumper;
        return initComplete | userRequestExit | autonOpMode.isStopRequested() | autonOpMode.isStarted();
    }

    @Override
    public void performStateActions() {
        // see if both roller wheels buttons are pressed
        boolean isAligned = !frontRollerTouch.getState() && !backRollerTouch.getState();
        if (isAligned && !wasAligned) {
            // robot was just brought into position, reset stopWatchCalibrate
            stopWatchCalibration.reset();
        } else if (isAligned && wasAligned && stopWatchCalibration.getElapsedTimeMillis() > alignDuration){
            // robot has been held in position for sufficient time, perform calibration
            ebotsImu.initEbotsImu(hardwareMap);
            ebotsImu.setFieldHeadingWhenInitializedDeg(0);
            initComplete = true;
        }

        if (isAligned){
            ebotsBlinkin.lightsYellow();
        } else{
            ebotsBlinkin.lightsRed();
        }

        wasAligned = isAligned;
        updateTelemetry();
    }

    @Override
    public void performTransitionalActions() {
        if (initComplete) {
            long duration = 3000;
            double cycleTime = 500;
            StopWatch stopWatch = new StopWatch();

            double currentCycle = Math.round(stopWatch.getElapsedTimeMillis() / cycleTime);
            boolean cycleOn = currentCycle % 2 == 0;
            while (!autonOpMode.isStarted() && !autonOpMode.isStopRequested() && stopWatch.getElapsedTimeMillis() < duration) {
                if (cycleOn) {
                    ebotsBlinkin.lightsGreen();
                } else {
                    ebotsBlinkin.lightsOff();
                }
                currentCycle = Math.round(stopWatch.getElapsedTimeMillis() / cycleTime);
                cycleOn = currentCycle % 2 == 0;
            }
        }
        ebotsBlinkin.lightsOff();

        autonOpMode.telemetry.addLine("Exiting " + this.getClass().getSimpleName());
        autonOpMode.telemetry.addData("InitComplete ",  initComplete);
        autonOpMode.telemetry.update();

        Log.d("EBOTS","Exiting " + this.getClass().getSimpleName() + ", initComplete: " + initComplete);
    }

    private void updateTelemetry(){
        Telemetry telemetry = autonOpMode.telemetry;
        String twoDec = "%.2f";
        telemetry.addData("Current Field Heading: ", String.format(twoDec, ebotsImu.getCurrentFieldHeadingDeg(true)));
        telemetry.addLine("Calibration Timer: " + stopWatchCalibration.toString());
        telemetry.addData("Front Touch Sensor Pressed: ", !frontRollerTouch.getState());
        telemetry.addData("Back Touch Sensor Pressed: ", !backRollerTouch.getState());
    }
}
