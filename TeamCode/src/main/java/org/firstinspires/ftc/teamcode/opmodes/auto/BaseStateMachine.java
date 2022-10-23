package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.PixyCam;
import org.firstinspires.ftc.teamcode.components.Vuforia;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutonomous;

import java.util.ArrayList;
import java.util.List;
@Autonomous(name = "BaseStateMachine", group = "Autonomous")
public class BaseStateMachine extends BaseAutonomous {
    // List of all states the robot could be in
    Sleeve teamAsset;
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;
    private int parkStep = 0;
    private int currentPos = 0;
    private int junctionStep = 0;

    public enum State {
        IDENTIFY_TARGET,
        DRIVE_TO_MEDIUM_JUNCTION,
        POSITION_ROBOT_AT_JUNCTION,
        PLACE_CONE,
        PARK,
        ALIGN_WITH_POLE,
        END_STATE,
        REVERSE_JUNCTION,
        LOGGING,
    }

    public enum Sleeve {
        DAVID,
        BRIAN,
        TEAM,
    }

    private final static String TAG = "BaseStateMachine";// Logging tag
    private State mCurrentState;                         // Current State Machine State.
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    /** Initializes State Machine
     */
    public void init() {
        super.init();
        this.msStuckDetectInit     = 15000;
        this.msStuckDetectInitLoop = 15000;
        // Starts state machine
        vuforia = new Vuforia(hardwareMap, Vuforia.CameraChoice.WEBCAM1);
        //pixyCam = hardwareMap.get(PixyCam.class, "sensor_color");
        newState(State.IDENTIFY_TARGET);


    }

    @Override
    public void init_loop() {
        if(vuforia == null){
            return;
        }
        telemetry.addData("signal sleeve?: ", vuforia.identifyTeamAsset());
        telemetry.update();

        identifySleeve();
    }

    private void identifySleeve() {
        int i = vuforia.identifyTeamAsset();
        if(i >= 0){
            teamAsset = Sleeve.values()[i];
        }
    }

    /**
     * State machine loop
     */
    @Override
    public void loop() {
        // Update telemetry each time through loop
        telemetry.addData("State", mCurrentState);
        telemetry.update();
        // Execute state machine
        switch (mCurrentState) {
            case IDENTIFY_TARGET:
                if (teamAsset == null) {
                    //drive forward slowly/10 inches and identify again
                    //backwards is forwards

                    if (driveSystem.driveToPosition(100, DriveSystem.Direction.BACKWARD, 0.2)) {
                        currentPos += 100;
                        identifySleeve();
                        teamAsset = Sleeve.BRIAN;
                    }
                    identifySleeve();
                    telemetry.addData("signal sleeve?: ", vuforia.identifyTeamAsset());

                } else {
                    newState(State.DRIVE_TO_MEDIUM_JUNCTION);
                }
                break;
            case DRIVE_TO_MEDIUM_JUNCTION:
                mediumJunction();
                break;
            case ALIGN_WITH_POLE:
                int offsetX;
                //PixyCam.Block block = pixyCam.GetBiggestBlock(3);
                //offsetX = pixyCam.offSetX();
                //create a function that will decrease speed based on how close offset is from 0
            case REVERSE_JUNCTION:
                reverseJunction();
                break;
            case PARK:
                park();
                break;
            case END_STATE:
                Log.d("parked", teamAsset.toString());
                //"david" left two squares, "brain" center two, "7330" right two squares

        }

    }

    /** Changes state to given state
     * @param newState state to change to
     */
    private void newState(State newState) {
        // Restarts the state clock as well as the state
        mStateTime.reset();
        mCurrentState = newState;
    }

    private void park() {
        if (parkStep == 0) {
            if (driveSystem.driveToPosition(440-currentPos, DriveSystem.Direction.BACKWARD, 0.3)) {
                parkStep++;
            }
        }
        if (parkStep == 1) {
            if (teamAsset == Sleeve.DAVID && driveSystem.driveToPosition(500, DriveSystem.Direction.RIGHT, 0.3)) {
                newState(State.END_STATE);
            }
            if (teamAsset == Sleeve.BRIAN) {
                newState(State.END_STATE);
            }
            if (teamAsset == Sleeve.TEAM && driveSystem.driveToPosition(500, DriveSystem.Direction.LEFT, 0.3)) {
                newState(State.END_STATE);
            }
        }
    }

    private void mediumJunction(){
        if(junctionStep == 0){
            if(driveSystem.driveToPosition(100, DriveSystem.Direction.BACKWARD, 0.4)){
                junctionStep++;
            }
        }
        if(junctionStep == 1){
            if(driveSystem.turn(-15, 0.2)){
                junctionStep++;
            }
        }
        if(junctionStep == 2){
            if(driveSystem.driveToPosition(20, DriveSystem.Direction.BACKWARD,0.4)) {
                newState(State.REVERSE_JUNCTION);
            }
        }
    }

    private void reverseJunction(){
        if(junctionStep == 2){
            if(driveSystem.driveToPosition(100, DriveSystem.Direction.FORWARD, 0.4)){
                junctionStep--;
            }
        }
        if(junctionStep == 1){
            if(driveSystem.turn(15, 0.2)){
                junctionStep--;
            }
        }
        if(junctionStep == 0){
            if(driveSystem.driveToPosition(20, DriveSystem.Direction.FORWARD,0.4)) {
                newState(State.PARK);
            }
        }
    }
}