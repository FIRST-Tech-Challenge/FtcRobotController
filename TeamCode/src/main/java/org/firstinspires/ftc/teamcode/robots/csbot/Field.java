package org.firstinspires.ftc.teamcode.robots.csbot;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.TauDriveTrain;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.PathLine;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.Vector2;

import java.util.Arrays;
import java.util.HashMap;

public class Field {

    public FieldThing[] objects = new FieldThing[37];

    public FieldThing[] closeRightPattern = new FieldThing[7];
    public FieldThing[] closeLeftPattern = new FieldThing[7];

    public FieldThing[] farRightPattern = new FieldThing[6];
    public FieldThing[] farLeftPattern = new FieldThing[6];
    public boolean isBlue;

    //todo decide on start position
    public Pose2d startPose = new Pose2d(0, 0, 0);

    public Pose2d targetCoordinate;

    public static double INCHES_PER_GRID = 23.5;
    public int fieldWidth = 12;

    HashMap<String, FieldThing[]> mapObjects = new HashMap<String, FieldThing[]>();

    ConeStack coneStackRight = new ConeStack("AllianceRightStack",2.83,2.45,0);
    ConeStack coneStackLeft = new ConeStack("AllianceLeftStack",-2.85,2.5,0);


    public Field(boolean isBlue){
        targetCoordinate = poseToCoordinates(startPose);
        this.isBlue = isBlue;
        if(isBlue){
            objects[0] = new FieldThing("AllianceTerminalClose",2.5,0.5,0);
            objects[1] = new FieldThing("AllianceTerminalFar",-2.5,5.5,0);
            objects[2] = new FieldThing("EnemyTerminalClose",-2.5,0.5,0);
            objects[3] = new FieldThing("EnemyTerminalFar",2.5,5.5,0);

        }else{
            objects[0] = new FieldThing("AllianceTerminalClose",-2.5,0.5,0);
            objects[1] = new FieldThing("AllianceTerminalFar",2.5,5.5,0);
            objects[2] = new FieldThing("EnemyTerminalClose",2.5,0.5,0);
            objects[3] = new FieldThing("EnemyTerminalFar",-2.5,5.5,0);
        }
        mapObjects.put("AllianceTerminal", new FieldThing[]{objects[0], objects[1]});
        mapObjects.put("EnemyTerminal", new FieldThing[]{objects[2], objects[3]} );

        objects[4] = coneStackLeft;
        objects[5] = coneStackRight;
        objects[6] = new FieldThing("EnemyLeftStack",-2.5,3.5,0);
        objects[7] = new FieldThing("EnemyRightStack",2.5,3.5,0);

        mapObjects.put("AllianceStack", new FieldThing[]{objects[4], objects[5]});
        mapObjects.put("EnemyStack", new FieldThing[]{objects[6], objects[7]} );

        objects[8] = new FieldThing("AllianceSubstationLeft",-0.3,0.3,0);
        objects[9] = new FieldThing("AllianceSubstationRight",0.3,0.3,0);

        mapObjects.put("AllianceSubstation", new FieldThing[]{objects[8]});
        mapObjects.put("EnemySubstation", new FieldThing[]{objects[9]} );


        for(int i = 0; i < 3 ; i++){
            for(int j = 0; j < 3; j++){
                objects[i*3+j+10] = new FieldThing("GroundStation"+(i*3+j+1),-2 + j*2,5-i*2,1);

            }
        }
        mapObjects.put("GroundStation", Arrays.copyOfRange(objects,10,18));

        objects[19] = new FieldThing("LowPole1",-1,5,2);
        objects[20] = new FieldThing("LowPole2",1,5,2);
        objects[21] = new FieldThing("LowPole3",-2,4,2);
        objects[22] = new FieldThing("LowPole4",2,4,2);
        objects[23] = new FieldThing("LowPole5",-2,2,2);
        objects[24] = new FieldThing("LowPole6",2,2,2);
        objects[25] = new FieldThing("LowPole7",-1,1,2);
        objects[26] = new FieldThing("LowPole8",1,1,2);

        mapObjects.put("LowPole", Arrays.copyOfRange(objects,19,26));

        objects[27] = new FieldThing("MidPole1",-1,4,3);
        objects[28] = new FieldThing("MidPole2",1,4,3);
        objects[29] = new FieldThing("MidPole3",-1,2,3);
        objects[30] = new FieldThing("MidPole4",1,2,3);

        mapObjects.put("MidPole", Arrays.copyOfRange(objects,27,30));

        objects[31] = new FieldThing("HighPole1",0,4,4);
        objects[32] = new FieldThing("HighPole2",-1,3,4);
        objects[33] = new FieldThing("HighPole3",1,3,4);
        objects[34] = new FieldThing("HighPole4",0,2,4);

        mapObjects.put("HighPole", Arrays.copyOfRange(objects,31,34));

        objects[35] = new FieldThing("AllianceSignal1",-1.5,1.5,-1);
        objects[36] = new FieldThing("AllianceSignal2",1.5,1.5,-1);

        mapObjects.put("AllianceSignal", Arrays.copyOfRange(objects,35,36));

        initPatterns();
    }

    public int getScoringTargetIndex(){
        return scoringTargetIndex;
    }

    public int getPatternIndex(){
        return  patternIndex;
    }

    int scoringTargetIndex = -1;

    public void incTarget(){
        switch (patternIndex){
            case 0:
                if(scoringTargetIndex < closeRightPattern.length-1){
                    scoringTargetIndex++;
                }else{
                    scoringTargetIndex = -1;
                }
                break;
            case 1:
                if(scoringTargetIndex < closeRightPattern.length-1){
                    scoringTargetIndex++;
                }else{
                    scoringTargetIndex = -1;
                }
                break;
            case 2:
                if(scoringTargetIndex < closeLeftPattern.length-1){
                    scoringTargetIndex++;
                }else{
                    scoringTargetIndex = -1;
                }
                break;
            case 3:
                if(scoringTargetIndex < closeLeftPattern.length-1){
                    scoringTargetIndex++;
                }else{
                    scoringTargetIndex = -1;
                }
                break;
            case 4:
                if(scoringTargetIndex < farRightPattern.length-1){
                    scoringTargetIndex++;
                }else{
                    scoringTargetIndex = -1;
                }
                break;
            case 5:
                if(scoringTargetIndex < farRightPattern.length-1){
                    scoringTargetIndex++;
                }else{
                    scoringTargetIndex = -1;
                }
                break;
            case 6:
                if(scoringTargetIndex < farLeftPattern.length-1){
                    scoringTargetIndex++;
                }else{
                    scoringTargetIndex = -1;
                }
                break;
            case 7:
                if(scoringTargetIndex < farLeftPattern.length-1){
                    scoringTargetIndex++;
                }else{
                    scoringTargetIndex = -1;
                }
                break;
        }
    }
    public void decTarget(){

        if(scoringTargetIndex > 0) scoringTargetIndex--;

    }

    int patternIndex = 0;

    public void incScoringPattern(){

        if(patternIndex < 4) patternIndex++;
        scoringTargetIndex = -1;
    }
    public void decScoringPattern(){

        if(patternIndex > 0) patternIndex--;
        scoringTargetIndex = -1;
    }

    FieldThing rightSubSource;
    FieldThing rightConeSource;

    FieldThing leftSubSource;
    FieldThing leftConeSource;

    public FieldThing getConeSource(){
        switch (patternIndex){
            case 0:
                return rightSubSource;
            case 1:
                return rightConeSource;
            case 2:
                return rightSubSource;
            case 3:
                return rightConeSource;
            case 4:
                return leftSubSource;
            case 5:
                return leftConeSource;
            case 6:
                return leftSubSource;
            case 7:
                return leftConeSource;

            default:
                return null;
        }
    }

    public String getPatternName(){
        switch (patternIndex){
            case 0:
                return "SUB_CLOSE_RIGHT";
            case 1:
                return "CONE_CLOSE_RIGHT";
            case 2:
                return "SUB_FAR_RIGHT";
            case 3:
                return "CONE_FAR_RIGHT";
            case 4:
                return "SUB_CLOSE_LEFT";
            case 5:
                return "CONE_CLOSE_LEFT";
            case 6:
                return "SUB_FAR_LEFT";
            case 7:
                return "CONE_FAR_LEFT";
            default:
                return "NULL";
        }
    }

    public FieldThing getPatternObject(){
        if(scoringTargetIndex == -1){
            scoringTargetIndex = 0;
        }
        switch (patternIndex){
            case 0:
                return closeRightPattern[scoringTargetIndex];
            case 1:
                return closeRightPattern[scoringTargetIndex];
            case 2:
                return farRightPattern[scoringTargetIndex];
            case 3:
                return farRightPattern[scoringTargetIndex];
            case 4:
                return closeLeftPattern[scoringTargetIndex];
            case 5:
                return closeLeftPattern[scoringTargetIndex];
            case 6:
                return farLeftPattern[scoringTargetIndex];
            case 7:
                return farLeftPattern[scoringTargetIndex];
            default:
                return null;
        }
    }

    public String getPattern(){
        return "";
    }

    public void initPatterns(){

        rightSubSource = objects[9];
        leftSubSource = objects[8];

        rightConeSource = objects[5];
        leftConeSource = objects[4];

        closeRightPattern[0] = objects[34];
        closeRightPattern[1] = objects[33];
        closeRightPattern[2] = objects[32];
        closeRightPattern[3] = objects[30];
        closeRightPattern[4] = objects[29];
        closeRightPattern[5] = objects[24];
        closeRightPattern[6] = objects[26];

        farRightPattern[0] = objects[33];
        farRightPattern[1] = objects[32];
        farRightPattern[2] = objects[31];
        farRightPattern[3] = objects[28];
        farRightPattern[4] = objects[27];
        farRightPattern[5] = objects[22];

        closeLeftPattern[0] = objects[34];
        closeLeftPattern[1] = objects[32];
        closeLeftPattern[2] = objects[33];
        closeLeftPattern[3] = objects[29];
        closeLeftPattern[4] = objects[30];
        closeLeftPattern[5] = objects[23];
        closeLeftPattern[6] = objects[25];

        farLeftPattern[0] = objects[32];
        farLeftPattern[1] = objects[33];
        farLeftPattern[2] = objects[31];
        farLeftPattern[3] = objects[27];
        farLeftPattern[4] = objects[28];
        farLeftPattern[5] = objects[21];
    }

    public ConeStack getRightConeStack(){
        return coneStackRight;
    }

    public ConeStack getLeftConeStack(){
        return coneStackLeft;
    }

    public ConeStack getConeStack(boolean rightConeStack){
        if(rightConeStack){
            return getRightConeStack();
        }else{
            return  getLeftConeStack();
        }
    }

    public static double getBearing(Pose2d pos, FieldThing thing){
        return Math.toDegrees(Math.atan2(thing.y() - pos.getY(), thing.x())-pos.getX());
    }

    public static Pose2d coordinatesToPose(Pose2d coordinate){
        return new Pose2d(
                coordinate.getX()* INCHES_PER_GRID + INCHES_PER_GRID /2,
                coordinate.getY()* INCHES_PER_GRID + INCHES_PER_GRID /2,
                0
        );
    }
    public void changeOwnership(int ID, boolean isBlue, boolean isRed){
        objects[ID].setOwnership(isBlue,isRed);
    }

    public double distanceToFieldObject(Pose2d currentCoordinate, FieldThing object){
        return Math.sqrt( Math.pow(object.x() - currentCoordinate.getX(), 2) + Math.pow(object.y() - currentCoordinate.getY(), 2));
    }
    public FieldThing getNearestObject(String name, Pose2d currentCoordinate){
        FieldThing[] objectsOfType = mapObjects.get(name);
        double minDistance  = distanceToFieldObject(currentCoordinate, objectsOfType[0]);
        int minIndex = 0;
        for(int i = 1; i < objectsOfType.length ; i++){
            double distance = distanceToFieldObject(currentCoordinate, objectsOfType[i]);
            if( distance < minDistance){
                minDistance = distance;
                minIndex = i;
            }

        }
        return objectsOfType[minIndex];
    }

    public int GetNearest(int h,Vector2 pos) {
        int minIndex = 0;
        double min = 999;
        switch (h) {
            case 1:
                for (int i = 10; i < 19; i++) {
                    if (Vector2.magnitude(objects[i].getPositionVec().subtract(pos)) < min) {
                        minIndex = i;
                    }
                }
                break;

            case 2:
                for (int i = 19; i < 27; i++) {
                    if (Vector2.magnitude(objects[i].getPositionVec().subtract(pos)) < min) {
                        minIndex = i;
                    }
                }
                break;

            case 3:
                for (int i = 27; i < 31; i++) {
                    if (Vector2.magnitude(objects[i].getPositionVec().subtract(pos)) < min) {
                        minIndex = i;
                    }
                }
                break;

            case 4:
                for (int i = 31; i < 35; i++) {
                    if (Vector2.magnitude(objects[i].getPositionVec().subtract(pos)) < min) {
                        minIndex = i;
                    }
                }

            case -1:
                for (int i = 35; i < 37; i++) {
                    if (Vector2.magnitude(objects[i].getPositionVec().subtract(pos)) < min) {
                        minIndex = i;
                    }
                }

            default:
                break;

        }
        return minIndex;
    }

    public void updateTargetPose(double dx, double dy, TauDriveTrain driveTrain){
        Pose2d newTarget = new Pose2d(targetCoordinate.getX() + dx, targetCoordinate.getY() + dy, 0);
        //if(newTarget.getX() > fieldWidth || newTarget.getX() < 0) return;

        //if(newTarget.getY() > fieldWidth || newTarget.getY() < 0) return;

        targetCoordinate = newTarget;
        driveTrain.setGridDriveStateMachine(getPathToTarget(driveTrain));
    }
    /**
     * returns a state machine that will drive the robot to target grid position
     * each path is composed of atmost a vertical component and a horizontal component.
     */
    public Pose2d poseToCoordinates(Pose2d currentPose){
        //todo
        return new Pose2d(
                (currentPose.getX() - INCHES_PER_GRID /2) / INCHES_PER_GRID,
                (currentPose.getY() - INCHES_PER_GRID /2) / INCHES_PER_GRID,
                0);

    }
    public boolean orthogonalPoses(Pose2d pose1, Pose2d pose2){
        return (Utils.approxEquals(pose1.getX(), pose2.getX())) || Utils.approxEquals(pose1.getY(), pose2.getY());
    }
    public double angleToPose(Pose2d startPose, Pose2d endPose){
        //todo
        return 0;
    }
    public double angleDistance( double a1, double a2){
        return Math.min(
                Math.abs(a2 - a1),
                Math.abs( (a2+Math.PI) % (2*Math.PI) - (a1+Math.PI) % (2*Math.PI) )
        );
    }
    public boolean facingTargetDirection(double angle1, Pose2d currentCoordinate){
        double dy = targetCoordinate.getY() - currentCoordinate.getY();
        double dx = targetCoordinate.getX() - currentCoordinate.getX();
        double a2 = Math.atan2(dy, dx);
        return angleDistance(angle1, a2) < Math.PI/4;

    }
    public double getMagnitude(Pose2d pose2d){
        return Math.sqrt( Math.pow(pose2d.getX(), 2) + Math.pow(pose2d.getY(), 2));
    }
    public double getVelocityDot(Pose2d velocity, Pose2d currentCoordinate, Pose2d targetCoor){
        return (velocity.getX() * (targetCoor.getX() - currentCoordinate.getX())
                + velocity.getY() * (targetCoor.getY() - currentCoordinate.getY())) / getMagnitude(currentCoordinate);
    }
    public Pose2d getIntermediatePose(Pose2d currentCoordinate, double heading){

        double dx = targetCoordinate.getX() - currentCoordinate.getX();
        double dy = targetCoordinate.getY() - currentCoordinate.getY();
        //if line is horizontal
        if(!(angleDistance(0, heading) < Math.PI/4)){
            return new Pose2d(targetCoordinate.getX(), (int)currentCoordinate.getY(),0);

        }

        return  new Pose2d( (int)currentCoordinate.getY(), targetCoordinate.getY(),0);


    }

    public static Pose2d convertToGrids(Pose2d pos){
        return  new Pose2d(pos.getX()/INCHES_PER_GRID, -pos.getY()/INCHES_PER_GRID, pos.getHeading());
    }

    public static Pose2d convertToInches(Pose2d pos){
        return  new Pose2d(pos.getX()*INCHES_PER_GRID, pos.getY()*INCHES_PER_GRID, pos.getHeading());
    }
    public double getHeadingToPose(Pose2d currentPose, Pose2d targetPose){
        return Math.atan2(
                targetPose.getY() - currentPose.getY(),
                targetPose.getX() - currentPose.getX()
        );
    }

    public StateMachine getPathToTarget(TauDriveTrain driveTrain){
        StateMachine state;

        Pose2d currentCoordinate = poseToCoordinates(driveTrain.getPoseEstimate());
        double currentHeading = driveTrain.getExternalHeading();
        Pose2d velocity = driveTrain.getPoseVelocity();
        //does the robot only need one component to make it
        if (orthogonalPoses(currentCoordinate, targetCoordinate)){
            //if robot is heading in the right direction already, then you only need one path
            if(facingTargetDirection(currentHeading, currentCoordinate)){

                state = Utils.getStateMachine(new Stage())
                        .addState(() ->
                            driveTrain.setPath(new PathLine(currentCoordinate,targetCoordinate,getVelocityDot(velocity,currentCoordinate,targetCoordinate), Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION))
                        )
                        .addState(() ->
                                driveTrain.followPath()
                        )
                        //todo add drivetrain
                        .build();
                return state;
            }
           // otherwise, just treat it as two copmonents
        }


        //it will require two components

            Pose2d intermediatePose = getIntermediatePose(currentCoordinate,currentHeading);
            double newHeading = getHeadingToPose(intermediatePose,targetCoordinate);
            state = Utils.getStateMachine(new Stage())
                    .addState(() ->
                            driveTrain.setPath(new PathLine(
                                    currentCoordinate,
                                    intermediatePose,
                                    getVelocityDot(velocity,currentCoordinate,intermediatePose),
                                    Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION
                            ))
                    )
                    .addState(() ->
                            driveTrain.followPath()
                    )
                    .addState(()-> driveTrain.turnUntilRads(newHeading))
                    .addState(() ->
                            driveTrain.setPath(new PathLine(
                                    intermediatePose,
                                    targetCoordinate,
                                    0,
                                    Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION
                            ))
                    )
                    .addState(() ->
                            driveTrain.followPath()
                    )
                    //todo add drivetrain
                    .build();
            return state;




    }

    public void goToStack(Robot robot){
        Pose2d currentPose = robot.driveTrain.getPoseEstimate();
        FieldThing nearestStack = getNearestObject("AllianceStack", poseToCoordinates(currentPose));

        StateMachine path = Utils.getStateMachine(new Stage())
                .addState(() -> robot.crane.calculateFieldTargeting( nearestStack.x()*fieldWidth, nearestStack.y() * fieldWidth, nearestStack.getHeight()))
                .addState(() -> robot.crane.goToTarget())
                .stateEndAction(() -> {robot.crane.setCurrentStateMachineToPickUp();})
                .build();
        robot.crane.setCurrentStateMachine(path);
    }
    public void goToHighPole(Robot robot){
        Pose2d currentPose = robot.driveTrain.getPoseEstimate();
        FieldThing nearestStack = getNearestObject("HighPole", poseToCoordinates(currentPose));

        StateMachine path = Utils.getStateMachine(new Stage())
                .addState(() -> robot.crane.calculateFieldTargeting( nearestStack.x()*fieldWidth, nearestStack.y() * fieldWidth, nearestStack.getHeight()))
                .addState(() -> robot.crane.goToTarget())
                .stateEndAction(() -> {robot.crane.setCurrentStateMachineToDropCone();})
                .build();
        robot.crane.setCurrentStateMachine(path);

    }
    public void goToMediumPole(Robot robot){
        Pose2d currentPose = robot.driveTrain.getPoseEstimate();
        FieldThing nearestStack = getNearestObject("MidPole", poseToCoordinates(currentPose));

        StateMachine path = Utils.getStateMachine(new Stage())
                .addState(() -> robot.crane.calculateFieldTargeting( nearestStack.x()*fieldWidth, nearestStack.y() * fieldWidth, nearestStack.getHeight()))
                .addState(() -> robot.crane.goToTarget())
                .stateEndAction(() -> {robot.crane.setCurrentStateMachineToDropCone();})
                .build();
        robot.crane.setCurrentStateMachine(path);
    }
    public void goToLowPole(Robot robot){
        Pose2d currentPose = robot.driveTrain.getPoseEstimate();
        FieldThing nearestStack = getNearestObject("LowPole", poseToCoordinates(currentPose));

        StateMachine path = Utils.getStateMachine(new Stage())
                .addState(() -> robot.crane.calculateFieldTargeting( nearestStack.x()*fieldWidth, nearestStack.y() * fieldWidth, nearestStack.getHeight()))
                .addState(() -> robot.crane.goToTarget())
                .stateEndAction(() -> {robot.crane.setCurrentStateMachineToDropCone();})
                .build();
        robot.crane.setCurrentStateMachine(path);
    }

    public void goToGroundStation(Robot robot){
        Pose2d currentPose = robot.driveTrain.getPoseEstimate();
        FieldThing nearestStack = getNearestObject("GroundStation", poseToCoordinates(currentPose));

        StateMachine path = Utils.getStateMachine(new Stage())
                .addState(() -> robot.crane.calculateFieldTargeting( nearestStack.x()*fieldWidth, nearestStack.y() * fieldWidth, nearestStack.getHeight()))
                .addState(() -> robot.crane.goToTarget())
                .stateEndAction(() -> {robot.crane.setCurrentStateMachineToDropCone();})
                .build();
        robot.crane.setCurrentStateMachine(path);
    }



}


