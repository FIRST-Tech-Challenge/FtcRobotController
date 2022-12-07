package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Components.CV.CVMaster;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFGamepad;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.util.IMU;

import java.util.ArrayList;

public class Field {
    private SampleMecanumDrive roadrun;
    private CVMaster cv;
    private RFGamepad gp;
    private IMU imu;
    private ArrayList<Integer> tileMovement = new ArrayList<>();
    private ArrayList<Trajectory> fullmovement = new ArrayList<>();
    double lookingDistance = 20.0, dropDistance = 12;
    double[] poleValues = {0, 0, 0, 0}, cameraPos = {4, 4, 0.85}, dropPosition = {0, 0, 0};
    double[] currentPosition = {0,0,0};
    Trajectory dropTrajectory;
    private Pose2d nexttrajStartPos;


    double[][][] poleCoords = {
            //row 1
            {{-47, 47}, {-47, 23.5}, {-47, 0}, {-47, -23.5}, {-47, -47}},
            //row 2
            {{-23.5, 47}, {-23.5, 23.5}, {-23.5, 0}, {-23.5, -23.5}, {-23.5, -47}},
            //row 3
            {{0, 47}, {0, 23.5}, {0, 0}, {0, -23.5}, {0, -47}},
            //row 4
            {{23.5, 47}, {23.5, 23.5}, {23.5, 0}, {23.5, -23.5}, {23.5, -47}},
            //row 5
            {{47, 47}, {47, 23.5}, {47.5, 0}, {47, -23.5}, {47, -47}}};

    double[][][] tileCoords = {
            //row 1
            {{-58.75, 58.75}, {-58.75, 35.25}, {-58.75, 11.75}, {-58.75, -11.75}, {-58.75, -35.25}, {-58.75, -58.75}},
            //row 2
            {{-35.25, 58.75}, {-35.25, 35.25}, {-35.25, 11.75}, {-35.25, -11.75}, {-35.25, -35.25}, {-58.75, -58.75}},
            //row 3
            {{-11.75, 58.75}, {11.75, 35.25}, {11.75, 11.75}, {11.75, -11.75}, {11.75, -35.25}, {-58.75, -58.75}},
            //row 4
            {{11.75, 58.75}, {11.75, 35.25}, {11.75, 11.75}, {11.75, -11.75}, {11.75, -35.25}, {-58.75, -58.75}},
            //row 5
            {{35.25, 58.75}, {35.25, 35.25}, {35.25, 11.75}, {35.25, -11.75}, {35.25, -35.25}, {-58.75, -58.75}},
            //row 6
            {{58.75, 58.75}, {58.75, 35.25}, {58.75, 11.75}, {58.75, -11.75}, {58.75, -35.25}, {-58.75, -58.75}}};

    public Field(SampleMecanumDrive p_roadrun, CVMaster p_cv, IMU p_imu, RFGamepad p_gp) {
        roadrun = p_roadrun;
        cv = p_cv;
        gp = p_gp;
        imu = p_imu;
    }

    //which tile bot at
    public double[][][] atTile() {
        // first set of coords is for which tile e.g. {A,1}(0,0), second is for {x,y} in inches


        return tileCoords;
    }

    //is robot looking at a pole
    public boolean lookingAtPole() {

            double[] coords = cv.rotatedPolarCoord();
            minDistPole();
            Pose2d curntPose = roadrun.getPoseEstimate();//.7,2.5
        curntPose = new Pose2d(curntPose.getX(), curntPose.getY(), imu.updateAngle());
        dropPosition[2] =curntPose.getHeading() - coords[0] + cameraPos[2];
            dropPosition[0] = curntPose.getX() + (coords[1]) * cos(dropPosition[2]) + cameraPos[0]* sin(curntPose.getHeading()) + cameraPos[1] * cos(curntPose.getHeading());
            dropPosition[1] = curntPose.getY() + (coords[1]) * sin(dropPosition[2]) - cameraPos[1]* sin(curntPose.getHeading()) + cameraPos[0] * cos(curntPose.getHeading());
            dropPosition[2] =atan2(-(curntPose.getY()-dropPosition[1]),-(curntPose.getX()-dropPosition[0]));
            op.telemetry.addData("polex", poleValues[0]);
            op.telemetry.addData("poley",poleValues[1]);
            op.telemetry.addData("cvtheta",coords[0]);
            op.telemetry.addData("cvdistance",coords[1]);
            op.telemetry.addData("cvheight", cv.poleSize());
            op.telemetry.addData("x", getDropPosition().getX());
            op.telemetry.addData("y", getDropPosition().getY());
            op.telemetry.addData("heading", (getDropPosition().getHeading()*180/PI));
//            roadrun.setPoseEstimate(new Pose2d(2 * roadrun.getPoseEstimate().getX() + (coords[1]) * cos(roadrun.getPoseEstimate().getHeading() + coords[0]) - poleCoords[(int) poleValues[0]][(int) poleValues[1]][0],
//                    2 * roadrun.getPoseEstimate().getY() + (coords[1]) * sin(roadrun.getPoseEstimate().getHeading() + coords[0]) - poleCoords[(int) poleValues[0]][(int) poleValues[1]][1],
//                    2 * roadrun.getPoseEstimate().getHeading() + coords[0] - poleValues[3]));
            return true;
    }
    public void closestDropPosition(boolean correct){
        Pose2d curntPose = roadrun.getPoseEstimate();
        curntPose = new Pose2d(curntPose.getX(), curntPose.getY(), imu.updateAngle());
        currentPosition = new double[]{curntPose.getX(), curntPose.getY(), curntPose.getHeading()};
        minDistPole();
        op.telemetry.addData("poleValues", poleValues[0] + "," + poleValues[1]);
        if(poleValues[0]!=0||poleValues[1]!=0) {
            double[] idealDropPos = {poleCoords[(int) poleValues[0]][(int) poleValues[1]][0] + cos(currentPosition[2]) * dropDistance,
                    poleCoords[(int) poleValues[0]][(int) poleValues[1]][1] + sin(currentPosition[2]) * dropDistance};
            double dist = sqrt(pow(currentPosition[0] - idealDropPos[0], 2) + pow(currentPosition[1] - idealDropPos[1], 2));
            Pose2d correctedPose = curntPose;
            double[] error = toPolar(new double[]{currentPosition[0]-idealDropPos[0],currentPosition[1]-idealDropPos[1]});
            if (dist > 1.5) {
                correctedPose = new Pose2d(idealDropPos[0] + cos(error[1]) * 1.5, idealDropPos[1] + sin(error[1]) * 1.5, currentPosition[2]);
                if(correct){
                roadrun.setPoseEstimate(correctedPose);
                }
            }
            op.telemetry.addData("idealDropPos:" + idealDropPos[0] + "," + idealDropPos[1] + ":", false);
            op.telemetry.addData("dist:", dist);
            op.telemetry.addData("correctedDropPos:" + correctedPose.getX() + "," + correctedPose.getY() + ":", false);
        }

    }
    public void updateTrajectory(){
        dropTrajectory = roadrun.trajectoryBuilder(roadrun.getPoseEstimate()).lineToLinearHeading(
                getDropPosition()).build();
    }
    public Trajectory getTrajectory(){
        return dropTrajectory;
    }
    public Pose2d getDropPosition(){
        return new Pose2d(dropPosition[0],dropPosition[1],dropPosition[2]);
    }

    //which pole bot looking at
    public double[] lookedAtPole() {
        // first two indexes of coords is for which tile e.g. {V,1}(0,0), second two idexes is for {r,theta} in inches/radians
        return poleValues;
    }

    // returns which tile is closest with first two coords, third index is distance in inches
    public double[] minDistTile() {
        double currentx = nexttrajStartPos.getX();
        double currenty = nexttrajStartPos.getY();

        double[] closestTile = {0, 0, 100000000};
        for (int columnnum = 0; columnnum < 6; columnnum++) {
            for (int rownum = 0; rownum < 6; rownum++) {
                double dist = Math.pow(tileCoords[columnnum][rownum][0] - currentx, 2) + Math.pow(tileCoords[columnnum][rownum][1] - currenty, 2);
                if (dist < closestTile[2]) {
                    closestTile[0] = columnnum;
                    closestTile[1] = rownum;
                    closestTile[2] = sqrt(dist);
                }
            }
        }
        return closestTile;
    }

    // returns which pole is closest with first two indexes as pole coords, third index is distance in inches, fourth index is angle in radians
    public double[] minDistPole() {
        double[] closestPole = {10, 10, 0, 0};
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                if (i % 2 == 0 && j % 2 == 0) {
                    continue;
                }
                if (inCloseSight(i, j)) {
                    return poleValues;
                } else {
                    //do nothing
                }
            }
        }
        return closestPole;
    }

    public double[] toPolar(double[] p_coords) {
        return new double[]{pow(p_coords[0] * p_coords[0] + p_coords[1] * p_coords[1], .5), atan2(p_coords[1], p_coords[0])};
    }

    public boolean inCloseSight(int p_i, int p_j) {
        double[] dist = {currentPosition[0] - poleCoords[p_i][p_j][0], currentPosition[1] - poleCoords[p_i][p_j][1]};
//        dist[0] += sin((roadrun.getPoseEstimate().getHeading() )) * cameraPos[0] - cos(roadrun.getPoseEstimate().getHeading() ) * cameraPos[1];
//        dist[1] += cos((roadrun.getPoseEstimate().getHeading() )) * cameraPos[0] + sin(roadrun.getPoseEstimate().getHeading() ) * cameraPos[1];
        double[] polarCoords = toPolar(dist);
        op.telemetry.addData("diff", polarCoords[0] +"," + abs((polarCoords[1] - currentPosition[2]) * 180.0 / PI+360) % 360 +"   |   "+p_i+"," + p_j);
        if (polarCoords[0] < lookingDistance && abs((polarCoords[1] - currentPosition[2]) * 180.0 / PI+360) % 360 < 22.5) {
            poleValues = new double[]{p_i, p_j, polarCoords[0], polarCoords[1] - currentPosition[2]+ PI};
            return true;
        } else {
            return false;
        }
    }
    //      270
    //      ||
    // 0 = ROBOT = 180
    //      ||
    //      90

        // TODO: (if u are inputting some sort of tile coords, you can access those through the tileCoords above)
    public Trajectory autoTileAim(int leftright, double tileX, double tileY, double tileT){
        Trajectory turnLeft = null; //initialize b4hand bc it be like that
        Trajectory turnRight = null; //^^^^
        //the outermost if statement checks if current angle is in between a margin of 30 deg from up/right/down/left
        double curT = nexttrajStartPos.getHeading(); //current angle
        if(curT < toRadians(315) && curT > toRadians(225)){ //up
            if(leftright == 0){ //left
                turnLeft = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY+11.75, curT)) //starting pos TODO: starting pos wont
                        // TODO: be the center of the tile
                        .splineToSplineHeading(new Pose2d(tileX + 5.875 , tileY - 5.875, //guessed values to get to pole
                                tileT + toRadians(30)), toRadians(270)) //turning left + end tangent
                        .build();
            }
            if(leftright == 1){ //right
                turnRight = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY+11.75, curT)) //starting pos
                        .splineToSplineHeading(new Pose2d(tileX - 5.875 , tileY - 5.875, //guessed values to get to pole
                                tileT - toRadians(30)), toRadians(270)) //turning right + end tangent
                        .build(); //gobeeldah
            }
        }
        else if(curT < Math.toRadians(225) && curT > Math.toRadians(135)){ //right
            if(leftright == 0){
                turnLeft = roadrun.trajectoryBuilder(new Pose2d(tileX+11.75, tileY, curT))
                        .splineToSplineHeading(new Pose2d(tileX - 5.875 , tileY - 5.875,
                                tileT + toRadians(30)), toRadians(180))
                        .build();

            }
            if(leftright == 1){
                turnRight = roadrun.trajectoryBuilder(new Pose2d(tileX+11.75, tileY, curT))
                        .splineToSplineHeading(new Pose2d(tileX - 5.875 , tileY + 5.875,
                                tileT - toRadians(30)), toRadians(180))
                        .build();
            }
        }
        else if(curT < Math.toRadians(135) && curT > Math.toRadians(45)){ //down
            if(leftright == 0){
                turnLeft = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY-11.75, curT))
                        .splineToSplineHeading(new Pose2d(tileX - 5.875 , tileY + 5.875,
                                tileT + toRadians(30)), toRadians(90))
                        .build();
            }
            if(leftright == 1){
                turnRight = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY-11.75, curT))
                        .splineToSplineHeading(new Pose2d(tileX + 5.875 , tileY + 5.875,
                                tileT - toRadians(30)), toRadians(90))
                        .build();
            }
        }
        else if(curT < Math.toRadians(45) && curT > Math.toRadians(315)){ //left
            if(leftright == 0){
                turnLeft = roadrun.trajectoryBuilder(new Pose2d(tileX-11.75, tileY, curT))
                        .splineToSplineHeading(new Pose2d(tileX + 5.875 , tileY + 5.875,
                                tileT + toRadians(30)), toRadians(0))
                        .build();
            }
            if(leftright == 1){
                turnRight = roadrun.trajectoryBuilder(new Pose2d(tileX-11.75, tileY, curT))
                        .splineToSplineHeading(new Pose2d(tileX + 5.875 , tileY - 5.875,
                                tileT - toRadians(30)), toRadians(0))
                        .build();
            }
        }

        //TODO: i want u to return a trajectory not run it, but ask warren first
        if(leftright == 0){ //left
            return turnLeft;
        }
        else{ //right
            return turnRight;
        }
    }

    public double[] checkD_PAD(int direction) {
        double[] movementchanges = {0, 0, 0, 0};
        if (direction == 1) {
            movementchanges[1] = -23.5;
            movementchanges[3] = toRadians(270);
        }
        else if (direction == 2) {
            movementchanges[0] = -23.5;
            movementchanges[2] = toRadians(270);
            movementchanges[3] = toRadians(180);
        }
        else if (direction == 3) {
            movementchanges[1] = 23.5;
            movementchanges[3] = toRadians(90);
        }
        else {
            movementchanges[0] = 23.5;
            movementchanges[2] = toRadians(90);
            movementchanges[3] = toRadians(0);
        }

        return movementchanges;
    }

    public Trajectory autoLateralTileGenerator(int index) {

        tileMovement = gp.getSequence();

        if(tileMovement.get(index) == 5 || tileMovement.get(index) == 6){
            //TODO: fill in once harry fix
        }

        double[] movements = checkD_PAD(tileMovement.get(index));
        Pose2d target = new Pose2d(nexttrajStartPos.getX() + movements[0], nexttrajStartPos.getY() + movements[1]
                , nexttrajStartPos.getHeading() + movements[2]);
        Trajectory onemove = roadrun.trajectoryBuilder(new Pose2d(nexttrajStartPos.getX(), nexttrajStartPos.getY(),
                        nexttrajStartPos.getHeading()))
                .splineToSplineHeading(new Pose2d(nexttrajStartPos.getX() + movements[0], nexttrajStartPos.getY() +
                        movements[1], nexttrajStartPos.getHeading() + movements[2]), movements[3])
                .addDisplacementMarker(() -> gp.removeSequenceElement())
                .build();

        return onemove;
    }

    public void autoTileAdjustment() {
        tileMovement = gp.getSequence();

        double[] currenttile = minDistTile();
        double centerxadjustment = 0;
        double centeryadjustment = 0;
        double forwardbackwardangleadjustment = 0;
        double leftrightangleadjustment = 0;

        if (tileMovement.get(0) == 1 || tileMovement.get(0) == 3) {
            if (tileMovement.get(0) == 1) {
                centeryadjustment = min(nexttrajStartPos.getY() - currenttile[1], -2.5);
            }

            else if (tileMovement.get(0) == 3) {
                centeryadjustment = max(nexttrajStartPos.getY() - currenttile[1], 2.5);
            }


            if (abs(nexttrajStartPos.getHeading() - toRadians(90)) < abs(nexttrajStartPos.getHeading() - toRadians(270))) {
                forwardbackwardangleadjustment = toRadians(90);
            }
            else {
                forwardbackwardangleadjustment = toRadians(270);
            }
        }
        else {

            if (tileMovement.get(0) == 2) {
                centerxadjustment = min(nexttrajStartPos.getX() - currenttile[0], -2.5);
            }

            else if (tileMovement.get(0) == 4) {
                centerxadjustment = max(nexttrajStartPos.getX() - currenttile[0], 2.5);
            }

            if (abs(nexttrajStartPos.getHeading()) < abs(nexttrajStartPos.getHeading() - toRadians(180))) {
                leftrightangleadjustment = toRadians(0);
            }
            else {
                leftrightangleadjustment = toRadians(180);
            }
        }


        Trajectory adjustment = roadrun.trajectoryBuilder(new Pose2d(nexttrajStartPos.getX(), nexttrajStartPos.getY(),
                        nexttrajStartPos.getHeading()))
                .splineToLinearHeading(new Pose2d(currenttile[0] + centerxadjustment, currenttile[1] +
                                centeryadjustment, forwardbackwardangleadjustment + leftrightangleadjustment),
                        toRadians(270))
                .build();


        //if adjusment is needed
        if(centerxadjustment == nexttrajStartPos.getX() - currenttile[0] && centeryadjustment == nexttrajStartPos.getY()
                - currenttile[1] && forwardbackwardangleadjustment + leftrightangleadjustment == nexttrajStartPos.getHeading()) {
            fullmovement.add(adjustment);
        }

        nexttrajStartPos = new Pose2d(currenttile[0] + centerxadjustment, currenttile[1] + centeryadjustment,
                forwardbackwardangleadjustment + leftrightangleadjustment);
    }

    public void autoMovement() {
        tileMovement = gp.getSequence();
        fullmovement.clear();
        nexttrajStartPos = getCurPos();
        TrajectorySequence finalmovements = null;
        autoTileAdjustment();
//        fullmovement.add(autoTileMovementMaster());
        if (tileMovement.size() == 1) {
            fullmovement.add(autoLateralTileGenerator(0));
            roadrun.followTrajectoryAsync(fullmovement.get(0));
        } else if (tileMovement.size() == 2) {
            for (int i = 0; i < 2; i++) {
                fullmovement.add(autoLateralTileGenerator(i));
            }
            finalmovements = roadrun.trajectorySequenceBuilder(new Pose2d(getCurPos().getX(),
                            getCurPos().getY(), getCurPos().getHeading()))
                    .addTrajectory(fullmovement.get(0))
                    .addTrajectory(fullmovement.get(1))
                    .build();
        } else if (tileMovement.size() == 3) {
            for (int i = 0; i < 3; i++) {
                fullmovement.add(autoLateralTileGenerator(i));
            }
            finalmovements = roadrun.trajectorySequenceBuilder(new Pose2d(getCurPos().getX(),
                            getCurPos().getY(), getCurPos().getHeading()))
                    .addTrajectory(fullmovement.get(0))
                    .addTrajectory(fullmovement.get(1))
                    .addTrajectory(fullmovement.get(2))
                    .build();
        } else if (tileMovement.size() == 4) {
            for (int i = 0; i < 4; i++) {
                fullmovement.add(autoLateralTileGenerator(i));
            }
            finalmovements = roadrun.trajectorySequenceBuilder(new Pose2d(getCurPos().getX(),
                            getCurPos().getY(), getCurPos().getHeading()))
                    .addTrajectory(fullmovement.get(0))
                    .addTrajectory(fullmovement.get(1))
                    .addTrajectory(fullmovement.get(2))
                    .addTrajectory(fullmovement.get(3))
                    .build();
        } else if (tileMovement.size() == 5) {
            for (int i = 0; i < 5; i++) {
                fullmovement.add(autoLateralTileGenerator(i));
            }
            finalmovements = roadrun.trajectorySequenceBuilder(new Pose2d(getCurPos().getX(),
                            getCurPos().getY(), getCurPos().getHeading()))
                    .addTrajectory(fullmovement.get(0))
                    .addTrajectory(fullmovement.get(1))
                    .addTrajectory(fullmovement.get(2))
                    .addTrajectory(fullmovement.get(3))
                    .addTrajectory(fullmovement.get(4))
                    .build();
        }

        roadrun.followTrajectorySequenceAsync(finalmovements);

        //TODO: uncomment when harry fixes his func
//        if (tileMovement.get(tileMovement.size() - 1) == 5 || tileMovement.get(tileMovement.size() - 1) == 6) {
//            fullmovement.add(autoTileAim());
//        }
    }
    public Pose2d getCurPos() {
        return roadrun.getPoseEstimate();
    }
}
