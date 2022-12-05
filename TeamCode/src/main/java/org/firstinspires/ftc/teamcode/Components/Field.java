package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Components.CV.CVMaster;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFGamepad;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.IMU;

import java.util.ArrayList;

public class Field {
    private SampleMecanumDrive roadrun;
    private CVMaster cv;
    private IMU imu;
    double lookingDistance = 20.0, dropDistance = -10;
    double[] poleValues = {0, 0, 0, 0}, cameraPos = {/*4, 4, 0.41887*/0,0,0}, dropPosition = {0, 0, 0};
    private RFGamepad gp;
    private ArrayList<Integer> tileMovement = new ArrayList<>();
    private ArrayList<Trajectory> fullmovement = new ArrayList<>();
    Trajectory dropTrajectory;

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
        imu=p_imu;
        gp = p_gp;
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
    public void closestDropPosition(){
        minDistPole();
        Pose2d curntPose = roadrun.getPoseEstimate();
        curntPose = new Pose2d(curntPose.getX(), curntPose.getY(), imu.updateAngle());
        op.telemetry.addData("poleValues", poleValues[0] + "," + poleValues[1]);
        if(poleValues[0]!=0||poleValues[1]!=0) {
            double[] idealDropPos = {poleCoords[(int) poleValues[0]][(int) poleValues[1]][0] - cos(curntPose.getHeading()) * dropDistance,
                    poleCoords[(int) poleValues[0]][(int) poleValues[1]][1] - sin(curntPose.getHeading()) * dropDistance};
            double dist = sqrt(pow(curntPose.getX() - idealDropPos[0], 2) + pow(curntPose.getY() - idealDropPos[1], 2));
            Pose2d correctedPose = curntPose;
            double[] error = toPolar(new double[]{curntPose.getX()-idealDropPos[0],curntPose.getY()-idealDropPos[1]});
            if (dist > 1.5) {
                correctedPose = new Pose2d(idealDropPos[0] + cos(error[1]) * 1.5, idealDropPos[1] + sin(error[1]) * 1.5, curntPose.getHeading());
//                roadrun.setPoseEstimate(correctedPose);
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
        double currentx = roadrun.getPoseEstimate().getX();
        double currenty = roadrun.getPoseEstimate().getY();

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
//        poleValues[0]=0;
//        poleValues[1]=3;
        return closestPole;
    }

    public double[] toPolar(double[] p_coords) {
        return new double[]{pow(p_coords[0] * p_coords[0] + p_coords[1] * p_coords[1], .5), atan2(p_coords[1], p_coords[0])};
    }

    public boolean inCloseSight(int p_i, int p_j) {
        double[] dist = {roadrun.getPoseEstimate().getX() - poleCoords[p_i][p_j][0], roadrun.getPoseEstimate().getY() - poleCoords[p_i][p_j][1]};
//        dist[0] += sin((roadrun.getPoseEstimate().getHeading() )) * cameraPos[0] - cos(roadrun.getPoseEstimate().getHeading() ) * cameraPos[1];
//        dist[1] += cos((roadrun.getPoseEstimate().getHeading() )) * cameraPos[0] + sin(roadrun.getPoseEstimate().getHeading() ) * cameraPos[1];
        double[] polarCoords = toPolar(dist);
        op.telemetry.addData("diff", polarCoords[0] +"," + abs((polarCoords[1] - imu.updateAngle()) * 180.0 / PI+360) % 360 +"   |   "+p_i+"," + p_j);
        if (polarCoords[0] < lookingDistance && abs((polarCoords[1] - imu.updateAngle()) * 180.0 / PI+360) % 360 < 22.5) {
            poleValues = new double[]{p_i, p_j, polarCoords[0], polarCoords[1] - imu.updateAngle()+ PI};
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
    public void autoTileAim(int leftright, double tileX, double tileY, double tileT){//TODO: u inputting tile center coords?
        // TODO: (if u are inputting some sort of tile coords, you can access those through the tileCoords above)
        Trajectory turnLeft = null; //initialize b4hand bc it be like that
        Trajectory turnRight = null; //^^^^
        //the outermost if statement checks if current angle is in between a margin of 30 deg from up/right/down/left
        double curT = roadrun.getPoseEstimate().getHeading(); //current angle
        if(curT < toRadians(315) && curT > toRadians(225)){ //up
            if(leftright == 0){ //left
                turnLeft = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY, curT)) //starting pos TODO: starting pos wont
                        // TODO: be the center of the tile
                        .splineToSplineHeading(new Pose2d(tileX + 5.875 , tileY - 17.625, //guessed values to get to pole
                                tileT + toRadians(30)), toRadians(270)) //turning left + end tangent
                        .build();
            }
            if(leftright == 1){ //right
                turnRight = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY, curT)) //starting pos
                        .splineToSplineHeading(new Pose2d(tileX - 5.875 , tileY - 17.625, //guessed values to get to pole
                                tileT - toRadians(30)), toRadians(270)) //turning right + end tangent
                        .build(); //gobeeldah
            }
        }
        if(curT < toRadians(225) && curT > toRadians(135)){ //right
            if(leftright == 0){
                turnLeft = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY, curT))
                        .splineToSplineHeading(new Pose2d(tileX - 17.625 , tileY - 5.875,
                                tileT + toRadians(30)), toRadians(180))
                        .build();

            }
            if(leftright == 1){
                turnRight = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY, curT))
                        .splineToSplineHeading(new Pose2d(tileX - 17.625 , tileY + 5.875,
                                tileT - toRadians(30)), toRadians(180))
                        .build();
            }
        }
        if(curT < toRadians(135) && curT > toRadians(45)){ //down
            if(leftright == 0){
                turnLeft = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY, curT))
                        .splineToSplineHeading(new Pose2d(tileX - 5.875 , tileY + 17.625,
                                tileT + toRadians(30)), toRadians(90))
                        .build();
            }
            if(leftright == 1){
                turnRight = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY, curT))
                        .splineToSplineHeading(new Pose2d(tileX + 5.875 , tileY + 17.625,
                                tileT - toRadians(30)), toRadians(90))
                        .build();
            }
        }
        if(curT < toRadians(45) && curT > toRadians(315)){ //left
            if(leftright == 0){
                turnLeft = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY, curT))
                        .splineToSplineHeading(new Pose2d(tileX + 17.625 , tileY + 5.875,
                                tileT + toRadians(30)), toRadians(0))
                        .build();
            }
            if(leftright == 1){
                turnRight = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY, curT))
                        .splineToSplineHeading(new Pose2d(tileX + 17.625 , tileY - 5.875,
                                tileT - toRadians(30)), toRadians(0))
                        .build();
            }
        }

        //TODO: i want u to return a trajectory not run it, but ask warren first
        if(leftright == 0){ //left
            roadrun.followTrajectoryAsync(turnLeft); //go to left junction
        }
        else if(leftright == 1){ //right
            roadrun.followTrajectory(turnRight); //go to right junction
        }
    }

    public void checkD_PAD(int direction, double[] movementchanges) {
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
    }

    public Trajectory autoLateralTileGenerator(int index) {

        tileMovement = gp.getSequence();

        double[] movements = {0, 0, 0, 0};

        checkD_PAD(tileMovement.get(index), movements);
        Trajectory onemove = roadrun.trajectoryBuilder(new Pose2d(getCurPos().getX(), getCurPos().getY(),
                        getCurPos().getHeading()))
                .splineToLinearHeading(new Pose2d(getCurPos().getX() + movements[0], getCurPos().getY() + movements[1]
                        + getCurPos().getHeading() + movements[2]), movements[3])
                .addDisplacementMarker(() -> gp.removeSequenceElement())
                .build();

        return onemove;
    }

    public Trajectory autoTileMovementMaster() {
        tileMovement = gp.getSequence();

        double[] currenttile = minDistTile();
        double centerxadjustment = 0;
        double centeryadjustment = 0;
        double forwardbackwardangleadjustment = 0;
        double leftrightangleadjustment = 0;

        if (tileMovement.get(0) == 1 || tileMovement.get(0) == 3) {
            if (Math.abs(currenttile[0] - getCurPos().getX()) > 0.5) {
                if ((getCurPos().getY() - currenttile[1] > 2.5 && tileMovement.get(0) == 1) ||
                        (getCurPos().getY() - currenttile[1] < 2.5 && tileMovement.get(0) == 3)) {
                    centerxadjustment = 2.5;
                }

            }

            if (Math.abs(getCurPos().getHeading() - toRadians(90)) < Math.abs(getCurPos().getHeading() -
                    toRadians(270))) {
                forwardbackwardangleadjustment = toRadians(90);
            }
            else {
                forwardbackwardangleadjustment = toRadians(270);
            }
        }
        else {
            if (Math.abs(currenttile[1] - getCurPos().getY()) > 0.5) {
                if ((getCurPos().getX() - currenttile[0] > 2.5 && tileMovement.get(0) == 2) ||
                        (getCurPos().getX() - currenttile[0] < 2.5 && tileMovement.get(0) == 4)) {
                    centerxadjustment = 2.5;
                }
            }

            if (Math.abs(getCurPos().getHeading()) < Math.abs(getCurPos().getHeading() - toRadians(180))) {
                leftrightangleadjustment = toRadians(0);
            }
            else {
                leftrightangleadjustment = toRadians(180);
            }
        }

        Trajectory adjustment = roadrun.trajectoryBuilder(new Pose2d(getCurPos().getX(), getCurPos().getY(),
                        getCurPos().getHeading()))
                .splineToLinearHeading(new Pose2d(currenttile[0] + centerxadjustment, currenttile[1] +
                                centeryadjustment, forwardbackwardangleadjustment + leftrightangleadjustment),
                        toRadians(270))
                .build();

        return adjustment;
    }

    public void autoMovement() {
        tileMovement = gp.getSequence();
        fullmovement.clear();
        fullmovement.add(autoTileMovementMaster());
        if (tileMovement.size() == 1) {
            fullmovement.add(autoLateralTileGenerator(0));
        }
        else if (tileMovement.size() == 2) {
            for (int i = 0; i < 2; i++) {
                fullmovement.add(autoLateralTileGenerator(i));
            }
        }
        else if (tileMovement.size() == 3) {
            for (int i = 0; i < 3; i++) {
                fullmovement.add(autoLateralTileGenerator(i));
            }
        }
        else if (tileMovement.size() == 4) {
            for (int i = 0; i < 4; i++) {
                fullmovement.add(autoLateralTileGenerator(i));
            }
        }
        else if (tileMovement.size() == 5) {
            for (int i = 0; i < 5; i++) {
                fullmovement.add(autoLateralTileGenerator(i));
            }
        }

        //TODO: uncomment when harry fixes his func
//        if (tileMovement.get(tileMovement.size() - 1) == 5 || tileMovement.get(tileMovement.size() - 1) == 6) {
//            fullmovement.add(autoTileAim());
//        }
    }

    public Pose2d getCurPos() {
        return roadrun.getPoseEstimate();
    }
}
