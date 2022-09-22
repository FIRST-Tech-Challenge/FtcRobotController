package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.AutonDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.DriveToEncoderTarget;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateStrafeToTouchWall implements EbotsAutonState{
    private EbotsAutonOpMode autonOpMode;
    private Telemetry telemetry;

    private int targetClicks;
    private long stateTimeLimit;
    private StopWatch stopWatch;
    private DriveToEncoderTarget motionController;

    private String logTag = "EBOTS";
    private boolean firstPass = true;
    private boolean isSpeedSlow = false;

    private DigitalChannel frontRollerTouch;
    private DigitalChannel backRollerTouch;


    public StateStrafeToTouchWall(EbotsAutonOpMode autonOpMode){
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");
        this.autonOpMode = autonOpMode;
        this.telemetry = autonOpMode.telemetry;
        motionController = new DriveToEncoderTarget(autonOpMode);

        targetClicks = 2500;
        stateTimeLimit = 4000;
        stopWatch = new StopWatch();
        int allianceSign = (AllianceSingleton.isBlue()) ? 1 : -1;
        motionController.strafe(90 * allianceSign, targetClicks);


        if (AllianceSingleton.isBlue()){
            frontRollerTouch = autonOpMode.hardwareMap.get(DigitalChannel.class, "leftFrontTouch");
            backRollerTouch = autonOpMode.hardwareMap.get(DigitalChannel.class, "leftBackTouch");
        } else {
            frontRollerTouch = autonOpMode.hardwareMap.get(DigitalChannel.class, "rightFrontTouch");
            backRollerTouch = autonOpMode.hardwareMap.get(DigitalChannel.class, "rightBackTouch");
        }

        Log.d(logTag, "Constructor complete");
    }

    private boolean isPressed(DigitalChannel digitalChannel){
        return !digitalChannel.getState();
    }

    @Override
    public boolean shouldExit() {
        if(firstPass){
            Log.d(logTag, "Inside shouldExit...");
            firstPass = false;
        }
        boolean stateTimedOut = stopWatch.getElapsedTimeMillis() > stateTimeLimit;
        boolean targetTravelCompleted = motionController.isTargetReached();

        boolean frontTouchPressed = isPressed(frontRollerTouch);
        boolean backTouchPressed = isPressed(backRollerTouch);
        boolean touchingWall = frontTouchPressed && backTouchPressed;
        if (stateTimedOut) Log.d(logTag, "Exited because timed out. ");
        if (targetTravelCompleted) Log.d(logTag, "Exited because travel completed. ");
        if (frontTouchPressed) Log.d(logTag, "Front Roller is contacting wall. ");
        if (backTouchPressed) Log.d(logTag, "Back Roller is contacting wall. ");
        if (touchingWall) Log.d(logTag, "Exiting because both rollers touching ");

        return stateTimedOut | targetTravelCompleted | touchingWall | !autonOpMode.opModeIsActive();
    }

    @Override
    public void performStateActions() {
        if (!isSpeedSlow && motionController.getAverageClicks() > targetClicks*0.8) {
            motionController.setSpeed(0.35);
            isSpeedSlow = true;
        }
        telemetry.addData("Avg Clicks", motionController.getAverageClicks());
        telemetry.addData("Position Reached", motionController.isTargetReached());
        telemetry.addLine(stopWatch.toString());
    }

    @Override
    public void performTransitionalActions() {
        Log.d(logTag, "Inside transitional Actions...");
        motionController.stop();
        motionController.logAllEncoderClicks();
        // Now pass the strafe clicks to the opmode for processing
        int avgClicksTraveled = motionController.getAverageClicks();
        autonOpMode.setStrafeClicksCollect(avgClicksTraveled);
        Log.d(logTag, "Setting strafe clicks to " + String.format("%d", avgClicksTraveled));

        telemetry.addLine("Exiting " + this.getClass().getSimpleName());
        telemetry.update();
    }
}
