package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.components.ArmSystem.Cone;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.PixyCam;

@Disabled
public class CompetitionAutonomous extends BaseCompetitionAutonomous {

    public static final int POLE_WIDTH = 40;;
    public static final int CONE_WIDTH = 100;
    private boolean park = false;
    public static final String TAG = "BaseCompetitionAutonomous";

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
    private double nowTime;
    private ElapsedTime time;
    /**
     * Initializes State Machine
     */
    public void init(TeamColor teamColor, TeamSide teamSide) {
        super.init();
        this.teamColor = teamColor;
        this.teamSide = teamSide;
        time = new ElapsedTime();
        sign = teamSide == TeamSide.LEFT ? -1: 1;
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
                        if (teamAsset == null) {
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
                    nowTime = time.seconds();
                    newState(State.ALIGN_WITH_POLE);
                }
                break;
            case ALIGN_WITH_POLE:
                if (align(PixyCam.YELLOW, POLE_WIDTH)) {
                    newState(State.PLACE_CONE);
                }
//                if ((time.seconds() - nowTime) >= 2) {
//                    newState(State.REVERSE_JUNCTION);
//                }
                break;
            case PLACE_CONE:
                if (scoreDaCone(ArmSystem.HIGH)) {
                    if(park){
                        newState(State.REVERSE_JUNCTION);
                    }
                    else{
                        newState(State.REVERSE_JUNCTION);
                    }
                }
                break;
            case DRIVE_TO_CONE:
                if (driveSystem.driveToPosition(360, DriveSystem.Direction.FORWARD, 0.8)) {
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
                if (drive_back_to_pole()) {
                    newState(State.ALIGN_WITH_POLE);
                }
                break;
            case REVERSE_JUNCTION:
                if (reverseJunction()){
                    if(park){
                        newState(State.PARK);
                    }else{
                        newState(State.DRIVE_TO_CONE);
                    }
                }
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
                    (teamAsset == Sleeve.TEAM && driveSystem.driveToPosition(680, DriveSystem.Direction.RIGHT, 0.3)) ||
                    (teamAsset == Sleeve.DAVID && driveSystem.driveToPosition(680, DriveSystem.Direction.LEFT, 0.3))) {
                return true;
            }
        }
        return false;
    }

    private boolean drive_to_junction() {
        switch (startPosition) {
            case START:
                if (step == 0) {
                    if (driveSystem.driveToPosition(1250 - currentPos, DriveSystem.Direction.FORWARD, .8)) {
                        step++;
                    }
                }
                if(step == 1){
                    if(driveSystem.driveToPosition(250, DriveSystem.Direction.BACKWARD, 0.4)){
                        step++;
                    }
                }
                if (step == 2) {
                    if (driveSystem.turn(-46 * sign, 0.5)) {
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
        int turn = park ? sign * 0 : sign * 90; //CHANGE!! real values should be 0 and 90
        if (step == 0) {
            if (driveSystem.driveToPosition(240, DriveSystem.Direction.BACKWARD, 0.4)) {
                step++;
            }
        }
//        if(step == 1) {
//            if (revertArm(0.5)) {
//                step += 1;
//            }
//        }
//        if(step == 2) {
//            if (driveSystem.turnAbsolute(turn, 0.5)) {
//                step = 0;
//                return true;
//            }
//        }
        if (step == 1) {
            boolean drive = driveSystem.turnAbsolute(turn, 0.4);
            boolean arm = revertArm(0.5);
            if (drive && arm) {
                step = 0;
                return true;
            }
        }
            return false;
        }


        private boolean intake_cone () {
            //TODO make applicable for multiple cones and add ternary
            if (step == 0 && armSystem.driveToLevel(Cone.FIVE.approach(), .3)) {
                step++;
            }
            if (step == 1 &&
                    driveSystem.driveToPosition(125, DriveSystem.Direction.FORWARD, 0.2)) {
                step++;
            }
            if (step == 2 && (armSystem.intake() || armSystem.driveToLevel(Cone.FIVE.grab(), 0.3))) {
                step++;
            }
            if (step == 3) {
                if (armSystem.intake()) { // Complete the intake process -- i.e. stop
                    step++;
                }
            }
            if (step == 4 && armSystem.driveToLevel(Cone.FIVE.clear(), 0.3)) {
                newState(State.DRIVE_BACK_TO_POLE);
                return true;
            }
            return false;
        }

        private boolean drive_back_to_pole () {
            if (step == 0) {
                if (driveSystem.driveToPosition(500, DriveSystem.Direction.BACKWARD, 0.3)) {
                    step++;
                }
            }

            if (step == 1) {
                if (driveSystem.turnAbsolute(sign * -45, 0.5)) {
                    step++;
                }
            }
            if (step == 2) {
                if (driveSystem.driveToPosition(30, DriveSystem.Direction.FORWARD, 0.3)) {
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

    @Override
    public void stop() {
        // Maybe?
        armSystem.armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armSystem.armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while(!revertArm(0.3)){

        }
    }
}