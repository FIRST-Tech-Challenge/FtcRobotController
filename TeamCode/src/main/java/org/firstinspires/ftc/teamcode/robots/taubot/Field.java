package org.firstinspires.ftc.teamcode.robots.taubot;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robots.taubot.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.taubot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.taubot.util.PathLine;
import org.firstinspires.ftc.teamcode.robots.taubot.util.Utils;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.Vector2;

import java.util.Arrays;
import java.util.HashMap;

public class Field {

    public FieldObject[] objects = new FieldObject[37];
    public boolean isBlue;

    //todo decide on start position
    public Pose2d startPose = new Pose2d(0, 0, 0);

    public Pose2d targetCoordinate;

    public static double INCHES_PER_GRID = 23.5;
    public int fieldWidth = 12;

    HashMap<String, FieldObject[]> mapObjects = new HashMap<String, FieldObject[]>();


    public Field(boolean isBlue){
        targetCoordinate = poseToCoordinates(startPose);
        this.isBlue = isBlue;
        if(isBlue){
            objects[0] = new FieldObject("AllianceTerminalClose",2.5,0.5,0);
            objects[1] = new FieldObject("AllianceTerminalFar",-2.5,5.5,0);
            objects[2] = new FieldObject("EnemyTerminalClose",-2.5,0.5,0);
            objects[3] = new FieldObject("EnemyTerminalFar",2.5,5.5,0);

        }else{
            objects[0] = new FieldObject("AllianceTerminalClose",-2.5,0.5,0);
            objects[1] = new FieldObject("AllianceTerminalFar",2.5,5.5,0);
            objects[2] = new FieldObject("EnemyTerminalClose",2.5,0.5,0);
            objects[3] = new FieldObject("EnemyTerminalFar",-2.5,5.5,0);
        }
        mapObjects.put("AllianceTerminal", new FieldObject[]{objects[0], objects[1]});
        mapObjects.put("EnemyTerminal", new FieldObject[]{objects[2], objects[3]} );

        objects[4] = new FieldObject("AllianceLeftStack",-2.5,2.5,0);
        objects[5] = new FieldObject("AllianceRightStack",2.5,2.5,0);
        objects[6] = new FieldObject("EnemyLeftStack",-2.5,3.5,0);
        objects[7] = new FieldObject("EnemyRightStack",2.5,3.5,0);

        mapObjects.put("AllianceStack", new FieldObject[]{objects[4], objects[5]});
        mapObjects.put("EnemyStack", new FieldObject[]{objects[6], objects[7]} );

        objects[8] = new FieldObject("AllianceSubstation",0,0.5,0);
        objects[9] = new FieldObject("EnemySubstation",0,5.5,0);

        mapObjects.put("AllianceSubstation", new FieldObject[]{objects[8]});
        mapObjects.put("EnemySubstation", new FieldObject[]{objects[9]} );


        for(int i = 0; i < 3 ; i++){
            for(int j = 0; j < 3; j++){
                objects[i*3+j+10] = new FieldObject("GroundStation"+(i*3+j+1),-2 + j*2,5-i*2,1);

            }
        }
        mapObjects.put("GroundStation", Arrays.copyOfRange(objects,10,18));

        objects[19] = new FieldObject("LowPole1",-1,5,2);
        objects[20] = new FieldObject("LowPole2",1,5,2);
        objects[21] = new FieldObject("LowPole3",-2,4,2);
        objects[22] = new FieldObject("LowPole4",2,4,2);
        objects[23] = new FieldObject("LowPole5",-2,2,2);
        objects[24] = new FieldObject("LowPole6",2,2,2);
        objects[25] = new FieldObject("LowPole7",-1,1,2);
        objects[26] = new FieldObject("LowPole8",1,1,2);

        mapObjects.put("LowPole", Arrays.copyOfRange(objects,19,26));

        objects[27] = new FieldObject("MidPole1",-1,4,3);
        objects[28] = new FieldObject("MidPole2",1,4,3);
        objects[20] = new FieldObject("MidPole3",-1,2,3);
        objects[30] = new FieldObject("MidPole4",1,2,3);

        mapObjects.put("MidPole", Arrays.copyOfRange(objects,27,30));

        objects[31] = new FieldObject("HighPole1",0,4,4);
        objects[32] = new FieldObject("HighPole2",-1,3,4);
        objects[33] = new FieldObject("HighPole3",1,3,4);
        objects[34] = new FieldObject("HighPole4",0,2,4);

        mapObjects.put("HighPole", Arrays.copyOfRange(objects,31,34));

        objects[35] = new FieldObject("AllianceSignal1",-1.5,1.5,-1);
        objects[36] = new FieldObject("AllianceSignal2",1.5,1.5,-1);

        mapObjects.put("AllianceSignal", Arrays.copyOfRange(objects,35,36));

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

    public double distanceToFieldObject(Pose2d currentCoordinate, FieldObject object){
        return Math.sqrt( Math.pow(object.x() - currentCoordinate.getX(), 2) + Math.pow(object.y() - currentCoordinate.getY(), 2));
    }
    public FieldObject getNearestObject(String name, Pose2d currentCoordinate){
        FieldObject[] objectsOfType = mapObjects.get(name);
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

    //todo: gets closest pole of a certain height
    public int GetNearest(int h,Vector2 pos) {
        int minIndex = 0;
        double min = 999;
        switch (h) {
            case 1:
                for (int i = 10; i < 19; i++) {
                    if (Vector2.magnitude(objects[i].getPosition().subtract(pos)) < min) {
                        minIndex = i;
                    }
                }
                break;

            case 2:
                for (int i = 19; i < 27; i++) {
                    if (Vector2.magnitude(objects[i].getPosition().subtract(pos)) < min) {
                        minIndex = i;
                    }
                }
                break;

            case 3:
                for (int i = 27; i < 31; i++) {
                    if (Vector2.magnitude(objects[i].getPosition().subtract(pos)) < min) {
                        minIndex = i;
                    }
                }
                break;

            case 4:
                for (int i = 31; i < 35; i++) {
                    if (Vector2.magnitude(objects[i].getPosition().subtract(pos)) < min) {
                        minIndex = i;
                    }
                }

            case -1:
                for (int i = 35; i < 37; i++) {
                    if (Vector2.magnitude(objects[i].getPosition().subtract(pos)) < min) {
                        minIndex = i;
                    }
                }

            default:
                break;

        }
        return minIndex;
    }

    public void updateTargetPose(double dx, double dy, DriveTrain driveTrain){
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
    public double getHeadingToPose(Pose2d currentPose, Pose2d targetPose){
        return Math.atan2(
                targetPose.getY() - currentPose.getY(),
                targetPose.getX() - currentPose.getX()
        );
    }
    public StateMachine getPathToTarget(DriveTrain driveTrain){
        Pose2d currentCoordinate = poseToCoordinates(driveTrain.getPoseEstimate());
        StateMachine state = Utils.getStateMachine(new Stage())
                .addState(() ->
                        driveTrain.setPath(new PathLine(new Pose2d(0,0,0),new Pose2d(2,0,0),0, 12, 10))
                )
                .addState(() ->
                        driveTrain.followPath()
                )
                //todo add drivetrain
                .build();
        return  state;
    }
    /*public StateMachine getPathToTarget(DriveTrain driveTrain){
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
                    .addState(()-> driveTrain.turnUntil(newHeading))
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
*/
    public void goToStack(Robot robot){
        Pose2d currentPose = robot.driveTrain.getPoseEstimate();
        FieldObject nearestStack = getNearestObject("AllianceStack", poseToCoordinates(currentPose));

        StateMachine path = Utils.getStateMachine(new Stage())
                .addState(() -> robot.crane.setTargets( nearestStack.x()*fieldWidth, nearestStack.y() * fieldWidth, nearestStack.getHeight()))
                .addState(() -> robot.crane.goToTarget())
                .stateEndAction(() -> {robot.crane.setCurrentStateMachineToPickUp();})
                .build();
        robot.crane.setCurrentStateMachine(path);
    }
    public void goToHighPole(Robot robot){
        Pose2d currentPose = robot.driveTrain.getPoseEstimate();
        FieldObject nearestStack = getNearestObject("HighPole", poseToCoordinates(currentPose));

        StateMachine path = Utils.getStateMachine(new Stage())
                .addState(() -> robot.crane.setTargets( nearestStack.x()*fieldWidth, nearestStack.y() * fieldWidth, nearestStack.getHeight()))
                .addState(() -> robot.crane.goToTarget())
                .stateEndAction(() -> {robot.crane.setCurrentStateMachineToDropCone();})
                .build();
        robot.crane.setCurrentStateMachine(path);

    }
    public void goToMediumPole(Robot robot){
        Pose2d currentPose = robot.driveTrain.getPoseEstimate();
        FieldObject nearestStack = getNearestObject("MidPole", poseToCoordinates(currentPose));

        StateMachine path = Utils.getStateMachine(new Stage())
                .addState(() -> robot.crane.setTargets( nearestStack.x()*fieldWidth, nearestStack.y() * fieldWidth, nearestStack.getHeight()))
                .addState(() -> robot.crane.goToTarget())
                .stateEndAction(() -> {robot.crane.setCurrentStateMachineToDropCone();})
                .build();
        robot.crane.setCurrentStateMachine(path);
    }
    public void goToLowPole(Robot robot){
        Pose2d currentPose = robot.driveTrain.getPoseEstimate();
        FieldObject nearestStack = getNearestObject("LowPole", poseToCoordinates(currentPose));

        StateMachine path = Utils.getStateMachine(new Stage())
                .addState(() -> robot.crane.setTargets( nearestStack.x()*fieldWidth, nearestStack.y() * fieldWidth, nearestStack.getHeight()))
                .addState(() -> robot.crane.goToTarget())
                .stateEndAction(() -> {robot.crane.setCurrentStateMachineToDropCone();})
                .build();
        robot.crane.setCurrentStateMachine(path);
    }

    public void goToGroundStation(Robot robot){
        Pose2d currentPose = robot.driveTrain.getPoseEstimate();
        FieldObject nearestStack = getNearestObject("GroundStation", poseToCoordinates(currentPose));

        StateMachine path = Utils.getStateMachine(new Stage())
                .addState(() -> robot.crane.setTargets( nearestStack.x()*fieldWidth, nearestStack.y() * fieldWidth, nearestStack.getHeight()))
                .addState(() -> robot.crane.goToTarget())
                .stateEndAction(() -> {robot.crane.setCurrentStateMachineToDropCone();})
                .build();
        robot.crane.setCurrentStateMachine(path);
    }



}


