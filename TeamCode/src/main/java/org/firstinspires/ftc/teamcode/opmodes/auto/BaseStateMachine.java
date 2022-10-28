package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.PixyCam;
import org.firstinspires.ftc.teamcode.components.Vuforia;

@Autonomous(name = "BaseStateMachine", group = "Autonomous")
public class BaseStateMachine extends BaseAutonomous {

    // List of all states the robot could be in
    public enum State {
        IDENTIFY_TARGET,
        DRIVE_TO_JUNCTION,
        PARK,
        ALIGN_WITH_POLE,
        END_STATE,
        GO_GET_CONE,
        ALIGN_WITH_CONE,
        REVERSE_JUNCTION
    }

    public enum Sleeve {
        DAVID,
        BRIAN,
        TEAM
    }
    
    public enum From {
        START, CONE_STACK
    }
    
    private Sleeve teamAsset;
    private From junctionPath;
    private int step = 0;
    private int currentPos = 0;
    private int encode;
    private PixyCam pixycam;

    private final static String TAG = "BaseStateMachine";// Logging tag
    private State mCurrentState;                         // Current State Machine State.

    /**
     * Initializes State Machine
     */
    public void init() {
        super.init();
        // Starts state machine
        vuforia = new Vuforia(hardwareMap, Vuforia.CameraChoice.WEBCAM1);
        pixycam = hardwareMap.get(PixyCam.class, "pixy");
        junctionPath = From.START;
        newState(State.REVERSE_JUNCTION);
    }

    @Override
    public void init_loop() {
        if (vuforia == null) {
            return;
        }
        telemetry.addData("signal sleeve?: ", vuforia.identifyTeamAsset());
        telemetry.update();

        identifySleeve();
    }

    private void identifySleeve() {
        int i = vuforia.identifyTeamAsset();
        if (i >= 0) {
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
                    newState(State.DRIVE_TO_JUNCTION);
                }
                break;
            case DRIVE_TO_JUNCTION:
                drive_to_junction();
                break;
            case ALIGN_WITH_POLE:
                align(3, 45, State.REVERSE_JUNCTION);
                break;
            case REVERSE_JUNCTION:
                reverseJunction();
                break;
            case GO_GET_CONE:
                getCone(); //dependent on team color
                break;
            case ALIGN_WITH_CONE:
                align(1, 80, State.END_STATE);
            case PARK:
                park();
                break;
            case END_STATE:
                //Log.d("parked", teamAsset.toString());
                //"david" left two squares, "brain" center two, "7330" right two squares

        }
        telemetry.update();
    }

    /**
     * Changes state to given state
     *
     * @param newState state to change to
     */
    private void newState(State newState) {
        mCurrentState = newState;
        step = 0;
    }

    private void park() {
        if (step == 0) {
            if (driveSystem.driveToPosition(440-currentPos, DriveSystem.Direction.BACKWARD, 0.3)) {
                step++;
            }
        }
        if (step == 1) {
            if (teamAsset == Sleeve.BRIAN ||
                    (teamAsset == Sleeve.TEAM && driveSystem.driveToPosition(500, DriveSystem.Direction.LEFT, 0.3)) ||
                    (teamAsset == Sleeve.DAVID && driveSystem.driveToPosition(500, DriveSystem.Direction.RIGHT, 0.3))) {
                newState(State.END_STATE);
            }
        }
    }

    private void drive_to_junction() {
        switch (junctionPath) {
            case START:
                if (step == 0) {
                    if (driveSystem.driveToPosition(950 - currentPos, DriveSystem.Direction.BACKWARD, 0.4)) {
                        step++;
                    }
                }
                if (step == 1) {
                    if (driveSystem.turn(-45, 0.2)) {
                        step = 0;
                        newState(State.ALIGN_WITH_POLE);
                    }
                }
                break;
            case CONE_STACK:

        }

    }

    private void reverseJunction() {
        if(step == 0){
            if (driveSystem.driveToPosition(40, DriveSystem.Direction.FORWARD, 0.4)){
                step++;
            }
        }
        if (step == 1) {
            if (driveSystem.turn(45, 0.2)) {
                step++;
            }
        }
        if (step == 2) {
            if (driveSystem.driveToPosition(450, DriveSystem.Direction.FORWARD, 0.4)) {
                newState(State.PARK);
            }
        }
    }

    private boolean alignX(int sign) {
        int offset = pixycam.offSetX(sign);
        telemetry.addData("offset", offset);
        if (offset > 20) {
            driveSystem.turn(60, 0.5);
        } else if (offset < -20) {
            driveSystem.turn(-60, 0.5);
        } else {
            driveSystem.setMotorPower(0);
            return true;
        }
        return false;
    }

    private boolean alignY(int desiredWidth){
        int align = pixycam.alignY(desiredWidth);// find actual desired width
        if (align > 5) {
            driveSystem.driveToPosition(100, DriveSystem.Direction.BACKWARD, 0.3);
        } else if (align < -5) {
            driveSystem.driveToPosition(100, DriveSystem.Direction.FORWARD, 0.3);
        } else {
            driveSystem.setMotorPower(0);
            return true;
            }
        return false;
    }

    private void getCone(){
        if(step == 0){
            if(driveSystem.driveToPosition(60, DriveSystem.Direction.FORWARD, 0.3)){
                step++;
            }
        }
        if(step == 1){
            if(driveSystem.turn(135, 0.5)){
                step++;
            }

        }
        if(step == 2) {
            if (driveSystem.driveToPosition(240, DriveSystem.Direction.BACKWARD, 0.5)) {
                step = 0;
                newState(State.END_STATE);
            }
        }
    }

    public void align(int sign, int desiredWidth, State nextState){
        if(step == 0){
            if(alignX(sign)){
                step++;
            }
        }
        if(step == 1){
            if(alignY(desiredWidth)){
                step = 0;
                newState(nextState);
            }
        }
    }

}