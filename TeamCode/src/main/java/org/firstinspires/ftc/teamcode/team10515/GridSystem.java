package org.firstinspires.ftc.teamcode.team10515;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class GridSystem {
    //grid orientated
    /*

        LANE 0
      0 1 0 1 0
  B     LANE 1
      1 2 3 2 1   R
        LANE 2
  L   0 3 0 3 0   E
        LANE 3
  U   1 2 3 2 1   D
        LANE 4
  E   0 1 0 1 0
        LANE 5
     */ //grid is symmetrical in every rotation and orientation

    // what do we want to know per lane
    // what poles are there, what side the poles are

    int[] blank = {};
    int[] pole01 = {0, 1, 0, 1, 0};
    int[] pole123 = {1, 2, 3, 2, 1};
    int[] pole03 = {0, 3, 0, 3, 0};
    int[][][] grid = {
            {blank, pole01, {0}},
            {pole01, pole123, {1}},
            {pole123, pole03, {2}},
            {pole03, pole123, {3}},
            {pole123, pole01, {4}},
            {pole01, blank, {5}}
    };

    // also want to know the exact spot of midpoint on tile and midpoint on seam and what pole is closest to it
    //these coordinates are NOT symmetrical from blue side and red side, i assume that its just negating the x and y values

    Vector2d[] lane0 = {new Vector2d(60, 60), new Vector2d(60, 48), new Vector2d(60, 36), new Vector2d(60, 24), new Vector2d(60, 12), new Vector2d(60, 0),
            new Vector2d(60, -12), new Vector2d(60, -24), new Vector2d(60, -36), new Vector2d(60, -48), new Vector2d(60, -60)};
    Vector2d[] lane1 = {new Vector2d(36, 60), new Vector2d(36, 48), new Vector2d(36, 36), new Vector2d(36, 24), new Vector2d(36, 12), new Vector2d(36, 0),
            new Vector2d(36, -12), new Vector2d(36, -24), new Vector2d(36, -36), new Vector2d(36, -48), new Vector2d(36, -60)};
    Vector2d[] lane2 = {new Vector2d(12, 60), new Vector2d(12, 48), new Vector2d(12, 36), new Vector2d(12, 24), new Vector2d(12, 12), new Vector2d(12, 0),
            new Vector2d(12, -12), new Vector2d(12, -24), new Vector2d(12, -36), new Vector2d(12, -48), new Vector2d(12, -60)};
    Vector2d[] lane3 = {new Vector2d(-12, 60), new Vector2d(-12, 48), new Vector2d(-12, 36), new Vector2d(-12, 24), new Vector2d(-12, 12), new Vector2d(-12, 0),
            new Vector2d(-12, -12), new Vector2d(-12, -24), new Vector2d(-12, -36), new Vector2d(-12, -48), new Vector2d(-12, -60)};
    Vector2d[] lane4 = {new Vector2d(-36, 60), new Vector2d(-36, 48), new Vector2d(-36, 36), new Vector2d(-36, 24), new Vector2d(-36, 12), new Vector2d(-36, 0),
            new Vector2d(-36, -12), new Vector2d(-36, -24), new Vector2d(-36, -36), new Vector2d(-36, -48), new Vector2d(-36, -60)};
    Vector2d[] lane5 = {new Vector2d(-60, 60), new Vector2d(-60, 48), new Vector2d(-60, 36), new Vector2d(-60, 24), new Vector2d(-60, 12), new Vector2d(-60, 0),
            new Vector2d(-60, -12), new Vector2d(-60, -24), new Vector2d(-60, -36), new Vector2d(-60, -48), new Vector2d(-60, -60)};

    public int findNearestPole(Vector2d robotPos){
        //Three things we need to know:
            //Which lane are we in (0-5)?
            //Are we closer to the poles on the left side of the lane or the right side of the lane?
            //Based on the above info and our y-coord, which pole are we closest to?

        robotPos.getX(); // 60 -> -60 TO 0 -> 5
        robotPos.getY(); // 60 -> 60 TO 0 -> 5
        double laneNumber = ((-robotPos.getX() + 60.0) * 5.0 / 120.0); //(-robotPos.getX() + 60) * (5 - 0) / (60 + 60) + 0
        int roundedLaneNumber = (int)Math.round(laneNumber);

        int closerToLeft;
        //do not necessarily need this for now
        if ((int)laneNumber == roundedLaneNumber){ //E.g. laneNumber is 2.3, then we are closer to the left side of lane 2
            closerToLeft = 1;
        }
        else {
            closerToLeft = 0;
        }

        double yLevel = ((-robotPos.getY() + 60.0) * 10.0 / 120.0); //(value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
        int roundedYLevel = (int)Math.round(yLevel); //Odd numbers are seams, even numbers are midpoints of tiles

        int[] typeOfPoleRow = grid[roundedLaneNumber][closerToLeft]; //Can be blank, pole01, pole123, or pole03
        if (typeOfPoleRow == blank){
            return -1; //No pole near us
        }
        if (roundedYLevel % 2 == 1) {
            int index = (roundedYLevel - 1)/2; //Since rounded level is odd, subtracting 1 and dividing by 2 will be a number between 0 and 4
            int typeOfPole = typeOfPoleRow[index];
            return typeOfPole;
        }
        //If rounded level is even (robot at midpoint of tile), what should we do?
        //We can either say there is no pole near us or go to the height of the highest pole out of the four around us
        return -1; //No pole near us (robot at midpoint or logic failed)
    }

}
