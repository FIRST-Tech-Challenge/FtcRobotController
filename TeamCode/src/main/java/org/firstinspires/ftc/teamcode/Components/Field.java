package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
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
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.util.IMU;

import java.util.ArrayList;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Field {
    private SampleMecanumDrive roadrun;
    private CVMaster cv;
    private RFGamepad gp;
    private IMU imu;
    private ArrayList<Integer> tileMovement = new ArrayList<>();
    private ArrayList<double[]> fullMovement = new ArrayList<>();
    private ArrayList<double[]> queuedMovement = new ArrayList<>();

    private ArrayList<Boolean> reversals = new ArrayList<>();
    private ArrayList<Boolean> queuedReversals = new ArrayList<>();
    int[] currentTile = {0, 0}, extra = {0, 0};
    int directionIndex = 1;
    boolean isReversed = true, lastReversed = true, autoTele = false;
    boolean[] needsCompile = {false, false};
    double lookingDistance = 20.0, dropDistance = 12;
    double[] poleValues = {0, 0, 0, 0}, cameraPos = {4, 4, 0.85}, dropPosition = {0, 0, 0};
    double[] currentPosition = {0, 0, 0};
    Trajectory dropTrajectory;
    private Pose2d nexttrajStartPos;
    boolean doneLookin = false;
    Lock locker;
    Lock needsLocker;
    Pose2d polePos = new Pose2d(0, 0, 0);
    Pose2d conePos = new Pose2d(0, 0, 0);


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
            {{-35.25, 58.75}, {-35.25, 35.25}, {-35.25, 11.75}, {-35.25, -11.75}, {-35.25, -35.25}, {-35.25, -58.75}},
            //row 3
            {{-11.75, 58.75}, {-11.75, 35.25}, {-11.75, 11.75}, {-11.75, -11.75}, {-11.75, -35.25}, {-11.75, -58.75}},
            //row 4
            {{11.75, 58.75}, {11.75, 35.25}, {11.75, 11.75}, {11.75, -11.75}, {11.75, -35.25}, {11.75, -58.75}},
            //row 5
            {{35.25, 58.75}, {35.25, 35.25}, {35.25, 11.75}, {35.25, -11.75}, {35.25, -35.25}, {35.25, -58.75}},
            //row 6
            {{58.75, 58.75}, {58.75, 35.25}, {58.75, 11.75}, {58.75, -11.75}, {58.75, -35.25}, {58.75, -58.75}}};

    public Field(SampleMecanumDrive p_roadrun, CVMaster p_cv, IMU p_imu, RFGamepad p_gp) {
        roadrun = p_roadrun;
        cv = p_cv;
        gp = p_gp;
        imu = p_imu;
        locker = new ReentrantLock();
        needsLocker = new ReentrantLock();
    }

    //which tile bot at
    public double[][][] atTile() {
        // first set of coords is for which tile e.g. {A,1}(0,0), second is for {x,y} in inches


        return tileCoords;
    }

    //is robot looking at a pole
    public boolean lookingAtPole() {
        double[] coords = cv.rotatedPolarCoord();
//        coords[1]+=5;
        coords[1] -= 1;
        Pose2d pos = roadrun.getPoseEstimate();
        pos = new Pose2d(pos.getX(), pos.getY(), pos.getHeading() + coords[0] * PI / 180 + PI);
        polePos = new Pose2d(pos.getX() + cos(pos.getHeading()) * coords[1] + sin(pos.getHeading()), pos.getY() + sin(pos.getHeading()) * coords[1] + cos(pos.getHeading()), pos.getHeading());
        if (abs(coords[1]) < 5 && abs(coords[1]) > 0) {
            setDoneLookin(true);
        }
//        if(abs(pos.vec().distTo(roadrun.getCurrentTraj().end().vec()))<2){
//            setDoneLookin(true);
//        }
        logger.log("/RobotLogs/GeneralRobot", "polePos" + polePos);
        logger.log("/RobotLogs/GeneralRobot", "coords" + coords[0] + "," + coords[1]);

        if (abs(pos.vec().distTo(roadrun.getCurrentTraj().end().vec())) < 10 && abs(coords[1]) < 18 && coords[1] > 3 && (roadrun.getCurrentTraj() == null || abs(polePos.vec().distTo(roadrun.getCurrentTraj().end().vec())) < 5)) {
            return true;
        }
        return false;
    }

    public boolean lookingAtPoleTele() {
        double[] coords = cv.rotatedPolarCoord();
//        coords[1]+=5;
        coords[1] -= 1.0;
        Pose2d pos = roadrun.getPoseEstimate();
        pos = new Pose2d(pos.getX(), pos.getY(), pos.getHeading() + coords[0] * PI / 180 + PI);
        polePos = new Pose2d(pos.getX() + cos(pos.getHeading()) * coords[1] + sin(pos.getHeading()), pos.getY() + sin(pos.getHeading()) * coords[1] + cos(pos.getHeading()), pos.getHeading());
        if (abs(coords[1]) < 5 && abs(coords[1]) > 0) {
            setDoneLookin(true);
        }
//        if(abs(pos.vec().distTo(roadrun.getCurrentTraj().end().vec()))<2){
//            setDoneLookin(true);
//        }
        logger.log("/RobotLogs/GeneralRobot", "polePos" + polePos);
        logger.log("/RobotLogs/GeneralRobot", "coords" + coords[0] + "," + coords[1]);

        if (abs(coords[1]) < 18 && coords[1] > 3) {
            return true;
        }
        return false;
    }

    public boolean lookingAtCone() {
        double[] coords = cv.rotatedConarCoord();
//        coords[1]+=5;
        coords[1] +=0;
        Pose2d pos = roadrun.getPoseEstimate();
        pos = new Pose2d(pos.getX(), pos.getY(), pos.getHeading() + coords[0] * PI / 180);
        conePos = new Pose2d(pos.getX() + cos(pos.getHeading()) * coords[1], pos.getY() + sin(pos.getHeading()) * coords[1] + 1, pos.getHeading());
        if (abs(coords[1]) < 5 && abs(coords[1]) > -1) {
            setDoneLookin(true);
        }
        logger.log("/RobotLogs/GeneralRobot", "CONePos" + conePos);
        logger.log("/RobotLogs/GeneralRobot", "coords" + coords[0] + "," + coords[1]);
        if (abs(coords[1]) > 4&&coords[1]<30) {

            return true;
        }
        return false;
    }

    public Pose2d polePos() {
        return polePos;
    }

    public Pose2d conePos() {
        return conePos;
    }

    public boolean isDoneLookin() {
        return doneLookin;
    }

    public void setDoneLookin(boolean inpu) {
        doneLookin = inpu;
    }

    public void closestDropPosition(boolean correct) {
        Pose2d curntPose = roadrun.getPoseEstimate();
        curntPose = new Pose2d(curntPose.getX(), curntPose.getY(), imu.updateAngle());
        currentPosition = new double[]{curntPose.getX(), curntPose.getY(), curntPose.getHeading()};
        minDistPole();
        op.telemetry.addData("poleValues", poleValues[0] + "," + poleValues[1]);
        if (poleValues[0] != 0 || poleValues[1] != 0) {
            double[] idealDropPos = {poleCoords[(int) poleValues[0]][(int) poleValues[1]][0] + cos(currentPosition[2]) * dropDistance,
                    poleCoords[(int) poleValues[0]][(int) poleValues[1]][1] + sin(currentPosition[2]) * dropDistance};
            double dist = sqrt(pow(currentPosition[0] - idealDropPos[0], 2) + pow(currentPosition[1] - idealDropPos[1], 2));
            Pose2d correctedPose = curntPose;
            double[] error = toPolar(new double[]{currentPosition[0] - idealDropPos[0], currentPosition[1] - idealDropPos[1]});
            if (dist > 1.5) {
                correctedPose = new Pose2d(idealDropPos[0] + cos(error[1]) * 1.5, idealDropPos[1] + sin(error[1]) * 1.5, currentPosition[2]);
                if (correct) {
                    roadrun.setPoseEstimate(correctedPose);
                }
            }
            op.telemetry.addData("idealDropPos:" + idealDropPos[0] + "," + idealDropPos[1] + ":", false);
            op.telemetry.addData("dist:", dist);
            op.telemetry.addData("correctedDropPos:" + correctedPose.getX() + "," + correctedPose.getY() + ":", false);
        }

    }

    public void updateTrajectory() {
        dropTrajectory = roadrun.trajectoryBuilder(roadrun.getPoseEstimate()).lineToLinearHeading(
                getDropPosition()).build();
    }

    public Trajectory getTrajectory() {
        return dropTrajectory;
    }

    public Pose2d getDropPosition() {
        return new Pose2d(dropPosition[0], dropPosition[1], dropPosition[2]);
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
                    closestTile[2] = dist;
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
        op.telemetry.addData("diff", polarCoords[0] + "," + abs((polarCoords[1] - currentPosition[2]) * 180.0 / PI + 360) % 360 + "   |   " + p_i + "," + p_j);
        if (polarCoords[0] < lookingDistance && abs((polarCoords[1] - currentPosition[2]) * 180.0 / PI + 360) % 360 < 22.5) {
            poleValues = new double[]{p_i, p_j, polarCoords[0], polarCoords[1] - currentPosition[2] + PI};
            return true;
        } else {
            return false;
        }
    }


    // TODO: (if u are inputting some sort of tile coords, you can access those through the tileCoords above)
//    public Trajectory autoTileAim(int leftright, double tileX, double tileY, double tileT) {
//        Trajectory turnLeft = null; //initialize b4hand bc it be like that
//        Trajectory turnRight = null; //^^^^
//        //the outermost if statement checks if current angle is in between a margin of 30 deg from up/right/down/left
//        double curT = nexttrajStartPos.getHeading(); //current angle
//        if (curT < toRadians(315) && curT > toRadians(225)) { //up
//            if (leftright == 0) { //left
//                turnLeft = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY + 11.75, curT)) //starting pos TODO: starting pos wont
//                        // TODO: be the center of the tile
//                        .splineToSplineHeading(new Pose2d(tileX + 5.875, tileY - 5.875, //guessed values to get to pole
//                                tileT + toRadians(30)), toRadians(270)) //turning left + end tangent
//                        .build();
//            }
//            if (leftright == 1) { //right
//                turnRight = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY + 11.75, curT)) //starting pos
//                        .splineToSplineHeading(new Pose2d(tileX - 5.875, tileY - 5.875, //guessed values to get to pole
//                                tileT - toRadians(30)), toRadians(270)) //turning right + end tangent
//                        .build(); //gobeeldah
//            }
//        } else if (curT < Math.toRadians(225) && curT > Math.toRadians(135)) { //right
//            if (leftright == 0) {
//                turnLeft = roadrun.trajectoryBuilder(new Pose2d(tileX + 11.75, tileY, curT))
//                        .splineToSplineHeading(new Pose2d(tileX - 5.875, tileY - 5.875,
//                                tileT + toRadians(30)), toRadians(180))
//                        .build();
//
//            }
//            if (leftright == 1) {
//                turnRight = roadrun.trajectoryBuilder(new Pose2d(tileX + 11.75, tileY, curT))
//                        .splineToSplineHeading(new Pose2d(tileX - 5.875, tileY + 5.875,
//                                tileT - toRadians(30)), toRadians(180))
//                        .build();
//            }
//        } else if (curT < Math.toRadians(135) && curT > Math.toRadians(45)) { //down
//            if (leftright == 0) {
//                turnLeft = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY - 11.75, curT))
//                        .splineToSplineHeading(new Pose2d(tileX - 5.875, tileY + 5.875,
//                                tileT + toRadians(30)), toRadians(90))
//                        .build();
//            }
//            if (leftright == 1) {
//                turnRight = roadrun.trajectoryBuilder(new Pose2d(tileX, tileY - 11.75, curT))
//                        .splineToSplineHeading(new Pose2d(tileX + 5.875, tileY + 5.875,
//                                tileT - toRadians(30)), toRadians(90))
//                        .build();
//            }
//        } else if (curT < Math.toRadians(45) && curT > Math.toRadians(315)) { //left
//            if (leftright == 0) {
//                turnLeft = roadrun.trajectoryBuilder(new Pose2d(tileX - 11.75, tileY, curT))
//                        .splineToSplineHeading(new Pose2d(tileX + 5.875, tileY + 5.875,
//                                tileT + toRadians(30)), toRadians(0))
//                        .build();
//            }
//            if (leftright == 1) {
//                turnRight = roadrun.trajectoryBuilder(new Pose2d(tileX - 11.75, tileY, curT))
//                        .splineToSplineHeading(new Pose2d(tileX + 5.875, tileY - 5.875,
//                                tileT - toRadians(30)), toRadians(0))
//                        .build();
//            }
//        }
//
//        if (leftright == 0) { //left
//            return turnLeft;
//        } else { //right
//            return turnRight;
//        }
//    }
//
//    public double[] checkD_PAD(int direction) {
//        double[] movementchanges = {0, 0, 0, 0};
//        if (direction == 1) {
//            movementchanges[1] = -23.5;
//            movementchanges[3] = toRadians(270);
//        } else if (direction == 2) {
//            movementchanges[0] = -23.5;
//            movementchanges[2] = toRadians(270);
//            movementchanges[3] = toRadians(180);
//        } else if (direction == 3) {
//            movementchanges[1] = 23.5;
//            movementchanges[3] = toRadians(90);
//        } else {
//            movementchanges[0] = 23.5;
//            movementchanges[2] = toRadians(90);
//            movementchanges[3] = toRadians(0);
//        }
//
//        return movementchanges;
//    }
    public boolean shouldReverse(int index) {
        if (((currentPosition[2] + toRadians(450) + index * toRadians(90)) % toRadians(360)) > toRadians(270)) {
            lastReversed = true;
            return true;
        } else if (((currentPosition[2] + toRadians(450) + index * toRadians(90)) % toRadians(360)) < toRadians(90)) {
            lastReversed = false;
            return false;
        } else {
            return lastReversed;
        }
    }

    public void autoLateralTileGenerator() {
        int direction = tileMovement.get(tileMovement.size() - 1);
        directionIndex = direction;
        boolean shouldMove = true;
        double notTooDeep = 9;
        double[] derection = {0, 0};
        double[] endTangents = {0, Math.toRadians(270), Math.toRadians(180), Math.toRadians(90), Math.toRadians(0)};
        int[] curTile = currentTile;
        if (direction == 5 || direction == 6) {
            //pole
        } else {
            if (direction == 1) {
                if (curTile[1] != 5) {
                    currentTile[1]++;
                    if (extra[1] == -1) {
                        extra[1] = 0;
                    }
                } else if (extra[1] != 1) {
                    notTooDeep = -2.5;
                    extra[1] = 1;
                } else {
                    shouldMove = false;
                }
                derection[1] = -1;
            } else if (direction == 2) {
                if (curTile[0] != 0) {
                    currentTile[0]--;
                    if (extra[0] == 1) {
                        extra[0] = 0;
                    }
                } else if (extra[0] != -1) {
                    notTooDeep = -2.5;
                    extra[1] = -1;
                } else {
                    shouldMove = false;
                }
                derection[0] = -1;
            } else if (direction == 3) {
                if (curTile[1] != 0) {
                    currentTile[1]--;
                    if (extra[1] == 1) {
                        extra[1] = 0;
                    }
                } else if (extra[1] != -1) {
                    notTooDeep = -2.5;
                    extra[1] = -1;
                } else {
                    shouldMove = false;
                }
                derection[1] = 1;
            } else if (direction == 4) {
                if (curTile[0] != 5) {
                    currentTile[0]++;
                    if (extra[0] == -1) {
                        extra[0] = 0;
                    }
                } else if (extra[0] != 1) {
                    notTooDeep = -2.5;
                    extra[1] = 1;
                } else {
                    shouldMove = false;
                }
                derection[0] = 1;
            }
            double[] target = tileCoords[currentTile[0]][currentTile[1]];
            logger.log("/RobotLogs/GeneralRobot", "currentTile:" + currentTile[0] + "," + currentTile[1]);
            shouldReverse(direction);
            int isReversed = 0;
            if (lastReversed) {
                isReversed++;
            } else {

            }
            double[] move = new double[]{target[0] - derection[0] * notTooDeep, target[1] - derection[1] * notTooDeep,
                    endTangents[direction] + isReversed * toRadians(180), endTangents[direction]};
            if (shouldMove) {
                addMove(move, lastReversed);
                compiledTrajectory();
            }
            gp.removeSequenceElement();
        }
    }

    public void addMove(double[] move, boolean reverse) {
        fullMovement.add(move);
        reversals.add(reverse);
    }

    public void compiledTrajectory() {
        logger.log("/RobotLogs/GeneralRobot", "aNewCompile");
        if (!roadrun.isBusy()) {
            Pose2d startPos = roadrun.getPoseEstimate();
            TrajectorySequenceBuilder bob = roadrun.trajectorySequenceBuilder(startPos);
            for (int i = 0; i < fullMovement.size(); i++) {
                bob.setReversed(reversals.get(i));
                Pose2d target = new Pose2d(fullMovement.get(0)[0], fullMovement.get(0)[1], fullMovement.get(0)[2]);
                bob.splineToSplineHeading(target, fullMovement.get(i)[3]);
            }
            bob.addTemporalMarker(() -> {
                fullMovement.clear();
                reversals.clear();
                gp.clearSequence();
            });
            roadrun.followTrajectorySequenceAsync(bob.build());
        } else {
            Pose2d startPos = roadrun.getCurrentTraj().start();
            TrajectorySequenceBuilder bob = roadrun.trajectorySequenceBuilder(startPos);
            for (int i = 0; i < fullMovement.size(); i++) {
                if(i==0||reversals.get(i)!=reversals.get(i-1)) {
                    bob.setReversed(reversals.get(i));
                }
                Pose2d target = new Pose2d(fullMovement.get(i)[0], fullMovement.get(i)[1], fullMovement.get(i)[2]);
                bob.splineToSplineHeading(target, fullMovement.get(i)[3]);
            }
            bob.addTemporalMarker(() -> {
                fullMovement.clear();
                reversals.clear();
                gp.clearSequence();
            });
            roadrun.changeTrajectorySequence(bob.build());
        }

    }

    public void autoMovement() {
        tileMovement = gp.getSequence();
        nexttrajStartPos = getCurPos();
        currentPosition = new double[]{nexttrajStartPos.getX(), nexttrajStartPos.getY(), nexttrajStartPos.getHeading()};
        double[] minDistTile = minDistTile();
        nexttrajStartPos = null;
        if (fullMovement.isEmpty() && !roadrun.isBusy()) {
            logger.log("/RobotLogs/GeneralRobot", "currentTile:" + minDistTile[0] + "," + minDistTile[1]);
            currentTile = new int[]{(int) minDistTile[0], (int) minDistTile[1]};
            extra = new int[]{0, 0};
        }
        autoLateralTileGenerator();
        tileMovement = null;
        if (!roadrun.isBusy() && !fullMovement.isEmpty()) {
            compiledTrajectory();
            System.gc();
        }
        minDistTile = null;
    }

    public boolean isAutoTele() {
        return autoTele;
    }

    public void breakAutoTele() {
        if (locker.tryLock()) {
            try {
                locker.lock();
                fullMovement.clear();
                reversals.clear();
                gp.clearSequence();
                isReversed = true;
                roadrun.breakFollowing();
                autoTele = false;
                System.gc();
            } finally {
                locker.unlock();
            }
        }
    }

    public Pose2d getCurPos() {
        return roadrun.getPoseEstimate();
    }
}
