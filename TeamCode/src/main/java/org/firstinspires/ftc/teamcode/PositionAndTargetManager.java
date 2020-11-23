package org.firstinspires.ftc.teamcode;


//todo: position manager doesn't work, basically goes infinite as soon as there is any movement, heading is wack, but doesn't go infinite (change in recorded heading isn't realistic, doesn't change enough when rotation, changes when going straight
// in response to the above issue, the update method was completely rewritten, these changes will be tested soon
public class PositionAndTargetManager{
    /*info found: https://firstinspiresst01.blob.core.windows.net/first-game-changers/ftc/field-setup-guide.pdf starting page 8
     *        and: https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/game-manual-part-2-remote-events.pdf starting page 26
     * Coords format: x,y,z
     * Origin (0,0) : center of the 6x6(square) grid
     *      field x bounds [-1.79705m,+1.79705m]
     *      field y bounds [-1.79705m,+1.79705m]
     * z = height from the foam grid
     * x+ = toward red alliance station (right from audience area perspective)
     * x- = toward blue alliance station
     * y+ = toward the tower goal and power shot targets
     * y- = toward audience
     *
     * position of red targets (center of volume) (x,y,z)
     *  - power shot 1:     +0.09525m,      +1.79705m,    +0.784225m
     *  - power shot 2:     +0.28575m,      +1.79705m,    +0.784225m
     *  - power shot 3:     +0.47625m,      +1.79705m,    +0.784225m
     *  - high goal:        +0.8905875m,    +1.79705m,    +0.911225m
     *  - medium goal:      -0.8905875m,    +1.79705m,    +0.6873875m
     *  - low goal:         +0.8905875m,    +1.79705m,    +0.4318m
     *
     * position of blue targets (center of volume) (x,y,z)
     *  - power shot 1:     -0.09525m,      +1.79705m,    +0.784225m
     *  - power shot 2:     -0.28575m,      +1.79705m,    +0.784225m
     *  - power shot 3:     -0.47625m,      +1.79705m,    +0.784225m
     *  - high goal:        -0.8905875m,    +1.79705m,    +0.911225m
     *  - medium goal:      +0.8905875m,    +1.79705m,    +0.6873875m
     *  - low goal:         -0.8905875m,    +1.79705m,    +0.4318m
     *
     * Red starting pos:    +___, -___, 0m
     * blue starting pos:   -___, -___, 0m
     */
    /*
        [r]
        r = 0, power shot 1
        r = 1, power shot 2
        r = 2, power shot 3
        r = 3, high goal
        r = 4, medium goal
        r = 5, low goal
        [c]
        c = 0, x
        c = 1, y
        c = 2, z
     */
    //initializes assuming it's on team red, in the constructor, values will be changed as needed if it's on team blue
    double[][] targets = {
            /*x,y,z*/
            {0.09525,1.79705,0.784225},     /*power shot 1*/
            {0.28575,1.79705,0.784225},     /*power shot 2*/
            {0.47625,1.79705,0.784225},     /*power shot 3*/
            {0.8905875,1.79705,0.911225},   /*high goal*/
            {-0.8905875,1.79705,0.6873875}, /*medium goal*/
            {0.8905875,1.79705,0.4318}      /*low goal*/
    };

    /*
        [r]
        r = 0, x
        r = 1, y
        r = 2, z
     */
    //initializes assuming it's on team red, in the constructor, values will be changed as needed if it's on team blue
    double[] robotPosition = {1.79705, -1.79705};
    double robotHeading = Math.PI / 2; //heading relative to field, pi/2 = toward goals

    double[] targetPosition = new double[3];
    int currentTarget;

    double launchZone = 2.06375-1.79705/*+- 0.0254m*/;       //any position with a y coordinate less (maybe more than?) than launchZone is in the launch zone

    double metersPerCount;
    double robotWidth;

    byte powerShotsHit = 0;

    int previousLeftCounts = 0;
    int previousRightCounts = 0;
    ////////////////////////////// constructors //////////////////////////////
    /**
     * constructor for the PositionAndTargetManager class
     * use this constructor if no autonomous was run
     * this constructor assumes that the robot is in the corner, facing toward the wall with the goal targets
     * @param robot     passed to allow for some calculations, and access to certain robot dimensions
     * @param isTeamRed true if on team red, false otherwise, used to personalize the target array, and initial position for what team the robot is on
     */
    public PositionAndTargetManager(HardwareUltimateGoal robot, boolean isTeamRed) {
        //take some variables from the robot
        metersPerCount = robot.NADO_METERS_PER_COUNT;
        robotWidth          = robot.robotWidth;
        //flip some things for if the robot is on blue team
        if (!isTeamRed) {
            for (int r = 0; r < targets.length; r++) {
                targets[r][0] *= -1.0; //flip the x-axis if not on the red team
            }

            robotPosition[0] *= -1.0; //flip the x-axis if not on the red team
        }
    }

    /**
     * constructor for the PositionAndTargetManager class
     * use this constructor if an autonomous was run
     * @param robot         passed to allow for some calculations, and access to certain robot dimensions
     * @param initPosition  this array should be given by the readPosition() method of HardwareUltimateGoal
     * @param initHeading   this should be given by the readHeading() method of HardwareUltimateGoal
     * @param isTeamRed     true if on team red, false otherwise, used to personalize the target array for what team the robot is on
     */
    public PositionAndTargetManager(HardwareUltimateGoal robot, double[] initPosition, double initHeading, boolean isTeamRed) {
        //take some variables from the robot
        metersPerCount = robot.NADO_METERS_PER_COUNT;
        robotWidth          = robot.robotWidth;
        //flip some things for if the robot is on blue team
        if (!isTeamRed) {
            for (int r = 0; r < targets.length; r ++) {
                targets[r][0] *= -1.0; //flip the x-axis if not on the red team
            }
        }
        //set position and heading to given values
        robotPosition[0]   = initPosition[0];
        robotPosition[1]   = initPosition[1];
        robotHeading    = initHeading;
    }

    ////////////////////////////// update and calculate method //////////////////////////////
    //TODO: currently, this only uses the encoders of the drive motors, once separate odometry systems are installed, rewrite this code, basing it on the ideas of team wizard https://www.youtube.com/watch?v=cpdPtN4BDug
    public void update(int leftCounts, int rightCounts) {
        double headingChange = 0.0;

        int leftChange = leftCounts - previousLeftCounts;
        int rightChange = rightCounts - previousRightCounts;

        //positions
        double s1 = leftChange * metersPerCount;  // distance the left wheel traveled (m) (delta s1)
        double s2 = rightChange * metersPerCount; // distance the right wheel traveled (m) (delta s2)
        double s = (s1 + s2) / 2.0; // average distance travelled between the two wheels

        //Calculate Angle change, robot width may need to be adjusted, and must be accurate to a high degree
        headingChange = (leftChange - rightChange) / (robotWidth);

        //some house keeping
        previousLeftCounts = leftCounts;
        previousRightCounts = rightCounts;

        //using a professors ideas https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-186-mobile-autonomous-systems-laboratory-january-iap-2005/study-materials/odomtutorial.pdf
        robotPosition[0] += s * Math.cos(robotHeading);
        robotPosition[1] += s * Math.sin(robotHeading);
        robotHeading += headingChange;

        /*using Wizards ideas would involve,
        replacing line 149 (robotPosition[0] += s * Math.cos(robotHeading);) with robotPosition[0] += (p*Math.sin(robotHeading) + n*Math.cos(robotHeading)) * metersPerCount;
        line 150 (robotPosition[1] += s * Math.sin(robotHeading);
        and putting 151 (robotHeading += headingChange;) after line 146 (previousRightCounts = rightCounts;

        and putting these lines:
            double p = (leftChange + rightChange) / 2.0; // average encoder ticks, tracking robot as whole rather than one side
            double n = (leftChange-rightChange); //horizontalChange
        before line 149
         */

    }
    /**
     * acts as an update method, that also returns the value it is setting the target position to
     * @param timeSeconds  time elapsed during match, measured in seconds, used to differentiate between mid game and end game targets
     * @return the position of the target selected, it also sets targetPosition to these coordinates
     */
    public double[] bestTargetPosition(double timeSeconds) {
        double[] bestTarget = new double[3];

        int i = (int) (Math.random() * (targets.length - 1)); //random row of targets -1
        if (i < 3 || i >= 5) { //prevents the robot from shooting the powershots before endgame, and increased the chance of aiming to the high goal
            i = 3;
        } // at this point, i should be either 3 or 4
        //TODO: 10/19/2020 change the > to < depending on what the launch zone actually is
        if (robotPosition[1] > launchZone) { // if the robot is outside of the launchZone
            i = 5; //target low goal
        }

        bestTarget[0] = targets[i][0];
        bestTarget[1] = targets[i][1];
        bestTarget[2] = targets[i][2];

        currentTarget = i;
        if (timeSeconds >= 200 && (int) (Math.random()*3 + 1) == 2 && powerShotsHit <= 2) { //if it's the endgame,  and rng (1/3)
            bestTarget[0] = targets[powerShotsHit][0]; // cycle through the powershots
            bestTarget[1] = targets[powerShotsHit][1];
            bestTarget[2] = targets[powerShotsHit][2];

            currentTarget = powerShotsHit;
            powerShotsHit ++;
        }

        targetPosition[0] = bestTarget[0];
        targetPosition[1] = bestTarget[1];
        targetPosition[2] = bestTarget[2];

        return targetPosition;
    }

    ////////////////////////////// get methods //////////////////////////////
    /**
     * @return robotHeading:     the heading, in radians, that the robot is facing
     */
    public double getRobotHeading() {
        return robotHeading;
    }

    /**
     * @return robotPosition:   the position (x,y) of the robot on the field, measured in meters
     */
    public double[] getRobotPosition() {
        return robotPosition;
    }

    /**
     * @return targetPosition:  the position (x,y,z) of the current target, measured in meters
     */
    public double[] getTargetPosition() {
        return targetPosition;
    }

    /**
     * @return a description of the current target
     */
    public String getCurrentTarget() {
        /*
        r = 0, power shot 1
        r = 1, power shot 2
        r = 2, power shot 3
        r = 3, high goal
        r = 4, medium goal
        r = 5, low goal
         */
        String description;
        switch (currentTarget) {
            case 0:
                description = "power shot 1";
                break;
            case 1:
                description = "power shot 2";
                break;
            case 2:
                description = "power shot 3";
                break;
            case 3:
                description = "high goal";
                break;
            case 4:
                description = "medium goal";
                break;
            case 5:
                description = "low goal";
                break;
            default:
                description = "no target selected, or unknown";
                break;
        }
        return description;
    }
}