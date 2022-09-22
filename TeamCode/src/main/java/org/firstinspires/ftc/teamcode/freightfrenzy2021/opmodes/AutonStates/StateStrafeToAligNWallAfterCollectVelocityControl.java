package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.ebotsutil.Pose;
import org.firstinspires.ftc.teamcode.ebotsutil.PoseError;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateStrafeToAligNWallAfterCollectVelocityControl extends EbotsAutonStateVelConBase{
    private boolean firstPass = true;
    private boolean isSpeedSlow = false;

    private DigitalChannel frontRollerTouch;
    private DigitalChannel backRollerTouch;
    private final Pose startPose;


    public StateStrafeToAligNWallAfterCollectVelocityControl(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");

        if (AllianceSingleton.isBlue()){
            frontRollerTouch = autonOpMode.hardwareMap.get(DigitalChannel.class, "leftFrontTouch");
            backRollerTouch = autonOpMode.hardwareMap.get(DigitalChannel.class, "leftBackTouch");
        } else {
            frontRollerTouch = autonOpMode.hardwareMap.get(DigitalChannel.class, "rightFrontTouch");
            backRollerTouch = autonOpMode.hardwareMap.get(DigitalChannel.class, "rightBackTouch");
        }

        startPose = new Pose(currentPose.getX(),currentPose.getY(), currentPose.getHeadingDeg());

        // Must define
        motionController.setSpeed(Speed.SLOW);
        travelDistance = 6.0;
        travelDirectionDeg = AllianceSingleton.isBlue() ? 90.0 : -90.0;
        targetHeadingDeg = 0.0;

        initAutonState();
        setDriveTarget();

        stateTimeLimit = 2000L;    // add some extra time for slow travel and touch

        Log.d(logTag, "Constructor complete");
    }

    private boolean isPressed(DigitalChannel digitalChannel){
        return !digitalChannel.getState();
    }

    @Override
    public boolean shouldExit() {
        // standard conditions include opMode inactivated, travel complete, state timed out
        boolean standardExitConditions = super.shouldExit();
        boolean frontTouchPressed = isPressed(frontRollerTouch);
        boolean backTouchPressed = isPressed(backRollerTouch);
        boolean touchingWall = frontTouchPressed && backTouchPressed;
        if (frontTouchPressed) Log.d(logTag, "Front Roller is contacting wall. ");
        if (backTouchPressed) Log.d(logTag, "Back Roller is contacting wall. ");
        if (touchingWall) Log.d(logTag, "Exiting because both rollers touching ");

        return standardExitConditions| touchingWall;
    }

    @Override
    public void performStateActions() {
        super.performStateActions();
        double distanceTraveled = new PoseError(startPose, currentPose, autonOpMode).getMagnitude();

        if (!isSpeedSlow && distanceTraveled > travelDistance*0.8) {
            motionController.setSpeed(Speed.SLOW);
            Log.d(logTag, "Shifted to slower speed for wall touch at " +
                    String.format(twoDec, distanceTraveled) +
                    " of target travel " + String.format(twoDec, travelDistance));
            isSpeedSlow = true;
        }
    }

    @Override
    public void performTransitionalActions() {
        super.performTransitionalActions();
        Log.d(logTag, "Inside transitional Actions...");

        // Now pass the strafe clicks to the opmode for processing
        int avgClicksTraveled = motionController.getAverageClicks();
        autonOpMode.setStrafeClicksCollect(avgClicksTraveled);
        Log.d(logTag, "Setting strafe clicks to " + String.format(intFmt, avgClicksTraveled));

        telemetry.addLine("Exiting " + this.getClass().getSimpleName());
        telemetry.update();
    }
}
