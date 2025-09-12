package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Waypoint;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;

import java.util.ArrayList;


/* *********** What Pure Pursuit Is ************
 * Nice Articles On It:
 *  - * https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit *
 *  - https://docs.ftclib.org/ftclib/pathing/pure-pursuit
 *  - https://www.mathworks.com/help/nav/ug/pure-pursuit-controller.html
 *  - https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
 *  - https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
 *  - https://arxiv.org/html/2409.09848v1#:~:text=They%20then%20combined%20this%20improved,calculate%20the%20required%20steering%20angle.&text=The%20results%20showed%20significant%20improvements,across%20different%20paths%20and%20speeds
 *  - https://vinesmsuic.github.io/robotics-purepersuit/
 *
 * Simplified:
 * Ok so basically you have a path of waypoints (5 types)
 * which you want your bot to follow.
 *
 * The bot has something called the look ahead distance
 * (ofc you have to tune it) which is basically the goal
 * position the bot is trying to move to.
 *
 * The goal position (look ahead distance away from the bot)
 * is in the path of waypoints and is constantly being updated
 *
 * The velocity is assumed constant.
 *
 * This is kindof what it looks like:
 *
 *
 *       #-----#                # <---End
 *      /      ^ \             /
 *     / waypoint \           /
 *    #            #---------# <---\
 *    |                             \--General Waypoint
 *   [*] <-------------Start
 *    ^ The bot
 *
 * Small Look Ahead Distance (Bot Oscillates):
 *
 *            ----
 *           /    \
 *          |      \       /---\
 *      #-------------------------------#
 *     /    |        \    /       \/
 *     |   /          ----
 *    /   |
 *   #   /
 *      [*]
 *
 * Big Look Ahead Distance (Bot Heavily Cuts Corners):
 *
 *       #----------------------------#
 *      /          ------------------
 *     /     /---/
 *    /  /--/
 *   #  /
 *     [*]
 *
 * */

// yeah I have no idea if this works, but we might use its so its probably
// ----- READY TO TRANSFER ----- //
public class PurePursuit {

    private double lookAheadDist;
    private int lastFoundNdx = 0;
    private int nextPointNdx = 1;
    private ArrayList<Waypoint> waypoints;
    private double turnError = 0.0;

    private final double BOTWIDTH = 0.0; /*NTBD*/
    private final double BASICLOOKVAL = 0.0; /*NTBD*/
    private final double CORRELATIONSPEED = 0.0; /*NTBD*/
    private final double CORRELATIONCURVATURE = 0.0; /*NTBD*/
    private final double SPACING = 0.0; /*NTBD*/
    private final double WEIGHT_SMOOTH = 0.0; /*NTBD, Should be between 0.75 ~ 0.98*/
    private final double TOLERANCE = 0.001;

    private boolean injectedWaypoints = false;

    /*Initialized lookAheadDist to 5.0 CM and the waypoints
     * ArrayList is initialized empty*/
    public PurePursuit() {
        lookAheadDist = 5.0;
        waypoints = new ArrayList<Waypoint>();
    }

    /*Initializes the lookAheadDist to the one provided.
     * The waypoints ArrayList is initialized empty*/
    public PurePursuit(double lookAheadDist) {
        this.lookAheadDist = lookAheadDist;
        waypoints = new ArrayList<Waypoint>();
    }

    /*Initializes the lookAheadDist and waypoints ArrayList to the ones provided*/
    public PurePursuit(double lookAheadDist, ArrayList<Waypoint> waypoints) {
        this.lookAheadDist = lookAheadDist;
        this.waypoints = waypoints;
    }

    /*Returns the lookAheadDist*/
    public double getLookAheadDistance() {
        return lookAheadDist;
    }

    /*Sets the lookAheadDist to the one provided*/
    public void setLookAheadDistance(double lookAheadDist) {
        this.lookAheadDist = lookAheadDist;
    }

    /*Sets the waypoints Arraylist to the one provided*/
    public void setWaypoints(ArrayList<Waypoint> waypoints) {
        this.waypoints = waypoints;
    }

    /*Returns the waypoints ArrayList*/
    public ArrayList<Waypoint> getWaypoints() {
        return waypoints;
    }

    /*Changes the Waypoint at the given position to the new
     * Waypoint provided. Nothing changed if an invalid position is given*/
    public void changeWaypoint(int pos, Waypoint newWaypoint) {
        if(pos >= 0 && pos < waypoints.size()) {
            waypoints.set(pos, newWaypoint);
        }
    }

    /*Returns the Waypoint at a certain position.
     * Null returned for an invalid position given.*/
    public Waypoint getWaypoint(int pos) {
        return (pos >= 0 && pos < waypoints.size()) ? waypoints.get(pos) : null;
    }

    /*Makes the given Waypoint the new starting Waypoint
     * The old start is made into the type GENERAL.*/
    public void addStartWaypoint(Waypoint newStartWaypoint) {
        waypoints.add(0, newStartWaypoint);
        if(waypoints.size() > 2) {
            waypoints.get(1).setType(Waypoint.types.GENERAL);
        } else if (waypoints.size() == 2) {
            waypoints.get(1).setType(Waypoint.types.END);
        }
    }

    /*Makes the new Waypoint given the starting Waypoint
     * The type parameter is the new type of the old start Waypoint*/
    public void addStartWaypoint(Waypoint newStartWaypoint, Waypoint.types type) {
        waypoints.add(0, newStartWaypoint);
        if(waypoints.size() > 2) {
            waypoints.get(1).setType(type);
        } else if (waypoints.size() == 2) {
            waypoints.get(1).setType(Waypoint.types.END);
        }
    }

    /*Makes the given Waypoint the new ending Waypoint
     * The old end is made into the type GENERAL.*/
    public void addEndWaypoint(Waypoint newStartWaypoint) {
        waypoints.add(newStartWaypoint);
        if(waypoints.size() > 2) {
            waypoints.get(1).setType(Waypoint.types.GENERAL);
        } else if (waypoints.size() == 2) {
            waypoints.get(1).setType(Waypoint.types.START);
        }
    }

    /*Makes the new Waypoint given the ending Waypoint
     * The type parameter is the new type of the old end Waypoint*/
    public void addEndWaypoint(Waypoint newStartWaypoint, Waypoint.types type) {
        waypoints.add(0, newStartWaypoint);
        if(waypoints.size() > 2) {
            waypoints.get(waypoints.size()-1).setType(type);
        } else if (waypoints.size() == 2) {
            waypoints.get(0).setType(Waypoint.types.START);
        }
    }

    /* **********Path Funcs********** */
    public void injectWaypoints() {
        injectedWaypoints = true;
        for (int i=0; i<waypoints.size()-1; i++) {
            double[] vector = subtractPoints(waypoints.get(i+1).getPos(), waypoints.get(i).getPos());
            double numPtsFit = Math.ceil(magnitudeVector(vector)/SPACING);
            vector = multiplyVectors(normalizeVector(vector), SPACING);
            for (int j=1; j<numPtsFit; j++) {
                waypoints.add(i+1, new Waypoint(addVectors(waypoints.get(i).getPos(), multiplyVectors(vector, j))));
            }
        }
    }

    public void smoothPath() {
        if (!injectedWaypoints) {
            injectWaypoints();
        }
        double change = TOLERANCE;
        while (change >= TOLERANCE) {
            change = 0.0;
            for (int i=1; i<waypoints.size()-1; i++) {
                for (int j=0; j<2; j++) {
                    Waypoint waypoint = waypoints.get(i);
                    double aux = waypoint.getPos()[j];
                    switch(j) {
                        case 0:
                            waypoint.setX(waypoint.getX(DistanceUnit.CM) + WEIGHT_SMOOTH * (waypoints.get(i-1).getX(DistanceUnit.CM) + waypoints.get(i+1).getX(DistanceUnit.CM) - (2.0*waypoint.getX(DistanceUnit.CM))), DistanceUnit.CM);
                            break;
                        case 1:
                            waypoint.setY(waypoint.getY(DistanceUnit.CM) + WEIGHT_SMOOTH * (waypoints.get(i-1).getY(DistanceUnit.CM) + waypoints.get(i+1).getY(DistanceUnit.CM) - (2.0*waypoint.getY(DistanceUnit.CM))), DistanceUnit.CM);
                            break;
                    }
                    change += Math.abs(aux - waypoint.getPos()[j]);
                }
            }
        }
    }

    /* **********Fuctionality Funcs********** */

    /* *****Helper Funcs***** */


    /*Finds the distance between the given points, pt1 and pt2*/
    public double distanceBetweenPoints(double[] pt1, double[] pt2) {
        return Math.sqrt(Math.pow((pt2[0]-pt1[0]), 2) + Math.pow((pt2[1]-pt1[0]), 2));
    }

    public double getTurnError(EditablePose2D botPos, double[] goalPoint) {
        return Math.abs(Math.atan((botPos.getY(DistanceUnit.CM)-goalPoint[1])/(botPos.getX(DistanceUnit.CM)-goalPoint[1]))- botPos.getH());
    }

    public double[] subtractPoints(double[] pt1, double[] pt2) {
        return new double[] {pt1[0]-pt2[0], pt1[1]-pt2[1]};
    }

    public double magnitudeVector(double[] vector) {
        return Math.sqrt(vector[0]*vector[0] + vector[1]*vector[1]);
    }

    public double[] normalizeVector(double[] vector) {
        return new double[] {vector[0]/ magnitudeVector(vector), vector[1]/ magnitudeVector(vector)};
    }

    public double[] multiplyVectors(double[] vector, double val) {
        return new double[] {vector[0] * val, vector[1] * val};
    }

    public double[] addVectors(double[] pt, double[] vector) {
        return new double[] {pt[0] + vector[0], pt[1] + vector[1]};
    }

    /*public double[] divideVector(double[] vector, double val) {
        return new double[] {vector[0] / val, vector[1] / val};
    }*/

    public int increment(int val, int increment, int maxval) {
        return (val + increment <= maxval) ? val + increment : 0;
    }

    /* *****Main Processing***** */

    public void updateLookAheadDistance(double velocity) {
        lookAheadDist = BASICLOOKVAL + CORRELATIONSPEED*velocity + CORRELATIONCURVATURE*turnError;
    }

    /*Finds the intersections between a circle of radius LookAheadDist
    centered at the robots position and a line drawn between the 2 points.

             # <---Waypoint
              \
          -----\
        /       \
       |   [*]   * <---Potential Goal Point
        \       / \
          -----    \
                    #

    */
/*    public double[] getIntersections(EditablePose2D botPos, Waypoint pt1, Waypoint pt2) {
        double botX = botPos.getX(DistanceUnit.CM);
        double botY = botPos.getY(DistanceUnit.CM);

        double dx = (pt2.getX() - botX) - (pt1.getX() - botX);
        double dy = (pt2.getY() - botY) - (pt1.getY() - botY);
        double dr = Math.sqrt(dx*dx + dy*dy);
        double D = (pt1.getX() - botX)*(pt2.getY() - botY) - (pt2.getX() - botX)*(pt1.getY() - botY);
        double discriminant = (lookAheadDist*lookAheadDist) * (dr*dr) - D*D;

        if (discriminant >= 0 && dr != 0) {
            double plusMinusTermX = Math.signum(dy) * dx * Math.sqrt(discriminant);
            double plusMinusTermY = Math.abs(dy) * Math.sqrt(discriminant);

            double x1 = (D * dy + plusMinusTermX) / dr*dr;
            double x2 = (D * dy - plusMinusTermX) / dr*dr;
            double y1 = (- D * dx + plusMinusTermY) / dr*dr;
            double y2 = (- D * dx - plusMinusTermY) / dr*dr;

            double[] sol1 = new double[]{x1 + botX, y1 + botY};
            double[] sol2 = new double[]{x2 + botX, y2 + botY};

            double minX = Math.min(pt1.getX(), pt2.getX());
            double maxX = Math.max(pt1.getX(), pt2.getX());
            double minY = Math.min(pt1.getY(), pt2.getY());
            double maxY = Math.max(pt1.getY(), pt2.getY());

            boolean sol1InRange = minX <= sol1[0] && sol1[0] <= maxX && minY <= sol1[1] && sol1[1] <= maxY;
            boolean sol2InRange = minX <= sol2[0] && sol2[0] <= maxX && minY <=sol2[1] && sol2[1] <= maxY;

            double[] goalPt;

            if(sol1InRange || sol2InRange) {
                if(!sol2InRange || (sol1InRange && distanceBetweenPoints(sol1, pt2.getPos()) < distanceBetweenPoints(sol2, pt2.getPos()))) {
                    goalPt = sol1;
                } else {
                    goalPt =  sol2;
                }

                if(distanceBetweenPoints(goalPt, pt2.getPos()) < distanceBetweenPoints(new double[]{botX, botY}, pt2.getPos())) {
                    return new double[]{goalPt[0], goalPt[1], 0.0};
                } else {
                    return goalPt;
                }
            }
        }
        return new double[1];
    }*/

    /*Finds the best goal position for the bot to stay on track.
     *
     *                         #
     *                          \
     *                           \
     *                            \
     *         #-------------------#
     *        /
     *       * <---Best Goal Pos
     *      / \
     *     /  [*] <---Bot
     *    #
     *
     * */

    /*public double[] searchGoalPoint(EditablePose2D botPos) {
        double[] goalPoint = new double[]{};
        double[][] goalPoints = new double[][]{};
        int startingNdx = lastFoundNdx;

        for (int i=0; i<waypoints.size()-1; i++) {
            goalPoint = getIntersections(botPos, waypoints.get(startingNdx), waypoints.get(increment(startingNdx, 1, waypoints.size()-1)));
            if(goalPoint.length == 3) {
                lastFoundNdx = i;
                break;
            } else if(goalPoint.length == 2) {
                lastFoundNdx = i+1;
            } else {
                goalPoint = new double[]{waypoints.get(lastFoundNdx).getX(), waypoints.get(lastFoundNdx).getY()};
            }
        }
        return new double[] {goalPoint[0], goalPoint[1]};
    }*/

    public double[] findGoalPoint(EditablePose2D botPos) {
        double botX = botPos.getX(DistanceUnit.CM);
        double botY = botPos.getY(DistanceUnit.CM);

        int startingIndex = lastFoundNdx;

        double[] goalPt = new double[]{};
        ArrayList<double[]> goalPts = new ArrayList<double[]>();

        for (int i=0; i<waypoints.size()-1; i++){
            int startingIndexIncremented = increment(startingIndex, 1, waypoints.size() - 1);

            double x1 = waypoints.get(startingIndex).getX(DistanceUnit.CM) - botX;
            double y1 = waypoints.get(startingIndex).getY(DistanceUnit.CM) - botY;
            double x2 = waypoints.get(startingIndexIncremented).getX(DistanceUnit.CM) - botX;
            double y2 = waypoints.get(startingIndexIncremented).getY(DistanceUnit.CM) - botY;
            double dx = x2 - x1;
            double dy = y2 - y1;
            double dr = Math.sqrt(dx*dx + dy*dy);
            double D = x1 * y2 - x2 * y1;
            double discriminant = (lookAheadDist*lookAheadDist) * (dr*dr) - D*D;

            if (discriminant >= 0 && dr != 0){
                double plusMinusTermX = Math.signum(dy) * dx * Math.sqrt(discriminant);
                double plusMinusTermY = Math.abs(dy) * Math.sqrt(discriminant);

                double sol1X = (D * dy + plusMinusTermX) / (dr*dr);
                double sol1Y = (D * dy - plusMinusTermX) / (dr*dr);
                double sol2X = (- D * dx + plusMinusTermY) / (dr*dr);
                double sol2Y = (- D * dx - plusMinusTermY) / (dr*dr);

                double[] sol1 = new double[]{sol1X + botX, sol1Y + botY};
                double[] sol2 = new double[]{sol2X + botX, sol2Y + botY};

                double minX = Math.min(waypoints.get(startingIndex).getX(DistanceUnit.CM), waypoints.get(startingIndexIncremented).getX(DistanceUnit.CM));
                double minY = Math.min(waypoints.get(startingIndex).getY(DistanceUnit.CM), waypoints.get(startingIndexIncremented).getY(DistanceUnit.CM));
                double maxX = Math.max(waypoints.get(startingIndex).getX(DistanceUnit.CM), waypoints.get(startingIndexIncremented).getX(DistanceUnit.CM));
                double maxY = Math.max(waypoints.get(startingIndex).getY(DistanceUnit.CM), waypoints.get(startingIndexIncremented).getY(DistanceUnit.CM));

                int nextPoint = (waypoints.get(nextPointNdx).isAnchor()) ? nextPointNdx: startingIndexIncremented;

                boolean sol1InRange = minX <= sol1[0] && sol1[0] <= maxX && minY <= sol1[1] && sol1[1] <= maxY;
                boolean sol2InRange = minX <= sol2[0] && sol2[0] <= maxX && minY <=sol2[1] && sol2[1] <= maxY;

                if(sol1InRange || sol2InRange) {
                    if (!sol2InRange || (sol1InRange && distanceBetweenPoints(sol1, waypoints.get(nextPoint).getPos()) < distanceBetweenPoints(sol2, waypoints.get(nextPoint).getPos()))) {
                        goalPt = sol1;
                    } else {
                        goalPt = sol2;
                    }

                    goalPts.add(goalPt);

                    if(distanceBetweenPoints(waypoints.get(nextPoint).getPos(), botPos.getPos()) < lookAheadDist) {
                        nextPointNdx = increment(nextPointNdx, 1, waypoints.size()-1);
                        lastFoundNdx = increment(lastFoundNdx, 1, waypoints.size()-1);
                    }

                    if(distanceBetweenPoints(goalPt, waypoints.get(nextPoint).getPos()) < distanceBetweenPoints(botPos.getPos(), waypoints.get(nextPoint).getPos())) {
                        lastFoundNdx = startingIndex;
                        nextPointNdx = increment(lastFoundNdx, 1, waypoints.size()-1);
                    } else {
                        lastFoundNdx = startingIndexIncremented;
                        nextPointNdx = increment(lastFoundNdx, 1, waypoints.size()-1);
                    }

                } else {
                    goalPt = waypoints.get(lastFoundNdx).getPos();
                }
            }
            if(waypoints.get(startingIndex).isAnchor() && distanceBetweenPoints(waypoints.get(startingIndex).getPos(), botPos.getPos()) > lookAheadDist) { break; }
            startingIndex = startingIndexIncremented;
        }

        if (!goalPts.isEmpty()) {goalPt = goalPts.get(goalPts.size()-1);}

        return goalPt;
    }

    public double[] getWheelPowers(EditablePose2D botPos, double velocity) {
        double[] goalPoint = findGoalPoint(botPos);
        turnError = getTurnError(botPos, goalPoint);
        double turnVelocity = (BOTWIDTH*Math.sin(turnError)*velocity)/lookAheadDist;
        return new double[] {velocity-turnVelocity, velocity+turnVelocity};
    }
}
