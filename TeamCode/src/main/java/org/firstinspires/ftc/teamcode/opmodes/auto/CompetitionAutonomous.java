package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.params.DriveParams.IMU;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.IMUSystem;
import org.firstinspires.ftc.teamcode.components.PixyCam;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@Disabled
public class CompetitionAutonomous extends BaseCompetitionAutonomous {

    public static final int POLE_WIDTH = 50;
    public static final int CONE_WIDTH = 150;
    private boolean park = false;

    // List of all states the robot could be in
    private enum State {
        IDENTIFY_TARGET,
        DRIVE_TO_JUNCTION,
        ALIGN_WITH_POLE,
        PLACE_CONE,
        DRIVE_TO_CONE,
        ALIGN_WITH_CONE,
        INTAKE_CONE,
        DRIVE_BACK_TO_POLE,
        PARK,
        REVERSE_JUNCTION,
        END_STATE
    }

    public enum From {
        START, CONE_STACK
    }

    public enum TeamColor {
        BLUE,
        RED
    }

    public enum TeamSide {
        RIGHT,
        LEFT
    }

    private From startPosition;
    private State mCurrentState;                         // Current State Machine State.

    protected TeamColor teamColor;
    protected TeamSide teamSide;
    private int sign;
    /**
     * Initializes State Machine
     */
    public void init(TeamColor teamColor, TeamSide teamSide) {
        super.init();
        this.teamColor = teamColor;
        this.teamSide = teamSide;
        if (teamSide == TeamSide.LEFT) {
            sign = -1;
        } else { // if teamSide is right
            sign = 1;
        }
        startPosition = From.START;
        newState(State.IDENTIFY_TARGET);
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
                    if (driveSystem.driveToPosition(100, DriveSystem.Direction.FORWARD, 0.2)) {
                        currentPos += 100;
                        identifySleeve();
                        if(teamAsset == null){
                            teamAsset = Sleeve.BRIAN;
                        }
                    }
                    identifySleeve();
                    telemetry.addData("signal sleeve?: ", vuforia.identifyTeamAsset());

                } else {
                    newState(State.DRIVE_TO_JUNCTION);
                }
                break;
            case DRIVE_TO_JUNCTION:
                if (drive_to_junction()) {
                    newState(State.ALIGN_WITH_POLE);
                }
                break;
            case ALIGN_WITH_POLE:
                if (align(PixyCam.YELLOW, POLE_WIDTH)) {
                    newState(State.PLACE_CONE);
                }
                break;
            case PLACE_CONE:
                if (!park) {
                    if( scoreDaCone(ArmSystem.HIGH)){
                        newState(State.REVERSE_JUNCTION);
                    }
                } else {
                    if(scoreDaCone(ArmSystem.HIGH)) {
                        newState(State.END_STATE);
                    }
                }
                break;
            case DRIVE_TO_CONE:
                if (drive_to_cone()) { //dependent on team color
                    newState(State.ALIGN_WITH_CONE);
                }
                break;
            case ALIGN_WITH_CONE:

                int color = teamColor == TeamColor.BLUE ? PixyCam.BLUE : PixyCam.RED;
                if (align(color, CONE_WIDTH)) {
                    newState(State.INTAKE_CONE);
                }
                break;
            case INTAKE_CONE:
                if (intake_cone()) {
                    newState(State.DRIVE_BACK_TO_POLE);
                }
                break;
            case DRIVE_BACK_TO_POLE:
                if(drive_back_to_pole()){
                    newState(State.ALIGN_WITH_POLE);
                }
                break;
            case REVERSE_JUNCTION:
                if(reverseJunction())
                    newState(State.PARK);
                break;
            case PARK:
                if (park()) {
                    newState(State.END_STATE);
                }
                break;
            case END_STATE:
                //Log.d("parked", teamAsset.toString());
                //"david" left two squares, "brain" center two, "7330" right two squares

        }
        telemetry.update();
    }

    private boolean park() {
        if (step == 0) {
            if (teamAsset == Sleeve.BRIAN ||
                    (teamAsset == Sleeve.TEAM && driveSystem.driveToPosition(600, DriveSystem.Direction.RIGHT, 0.3)) ||
                    (teamAsset == Sleeve.DAVID && driveSystem.driveToPosition(600, DriveSystem.Direction.LEFT, 0.3))) {
                return true;
            }
        }
        return false;
    }

    private boolean drive_to_junction() {
        switch (startPosition) {
            case START:
                if (step == 0) {
                    if (driveSystem.driveToPosition(1250 - currentPos, DriveSystem.Direction.FORWARD, 0.4)) {
                        step++;
                    }
                }
                if(step == 1){
                    if(driveSystem.driveToPosition(250, DriveSystem.Direction.BACKWARD, 0.4)){
                        step++;
                    }
                }
                if (step == 2) {
                    if (driveSystem.turn(-45 * sign, 0.5)) {
                        step = 0;
                        return true;
                    }
                }
                break;
            case CONE_STACK:

        }
        return false;

    }

    private boolean reverseJunction() {
        if(step == 0){
            if (driveSystem.driveToPosition(220, DriveSystem.Direction.BACKWARD, 0.4)){
                step++;
            }
        }
        if (step == 1) {
            if (revertArm(0.2) && driveSystem.turnAbsolute(0, 0.5)) {
                step = 0;
                return true;
            }
        }
        return false;
    }



    private boolean drive_to_cone(){

        // Back up
        if(step == 0){
            if(driveSystem.driveToPosition(150, DriveSystem.Direction.BACKWARD, 0.3)){
                step++;
            }
        }

        // Rotate
        if(step == 1){
            if(driveSystem.turnAbsolute(90 * sign, 0.5)){
                step++;
            }

        }

        // Drive to cone
        if(step == 2) {
            if (driveSystem.driveToPosition(360, DriveSystem.Direction.FORWARD, 0.5)) {
                return true;
            }
        }
        return false;
    }

    private boolean intake_cone() {
        return false;
    }

    private boolean drive_back_to_pole(){
        if(step == 0){
            if(driveSystem.driveToPosition(350, DriveSystem.Direction.BACKWARD, 0.3)){
                step++;
            }
        }

        if(step == 1){
            if(driveSystem.turn(sign* -135, 0.3)){
                step++;
            }
        }
        if(step == 2){
            if(driveSystem.driveToPosition(30, DriveSystem.Direction.FORWARD, 0.3)){
                step = 0;
                park = true;
                return true;
            }
        }
        return false;
    }
    /**
     * Changes state to given state
     *
     * @param newState state to change to
     */
    protected void newState(State newState) {
        mCurrentState = newState;
        step = 0;
    }
}