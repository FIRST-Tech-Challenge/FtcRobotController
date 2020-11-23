package org.firstinspires.ftc.teamcode;

/*
 * this will be the main class for the aim assist.
 * it should be able to take an input consisting of the robots position, heading, velocity(?), acceleration(?), and the position
 * of the target and be able to output the heading, and pitch required for the robot to launch the ring into the goal
 *
 * parameters with a ? before the description may not be needed.
 *
 * param   rPosition         a double array with two indexs recording the position of the robot (x,y) relative to the starting position.
 * param   rHeading              the direction that the robot is facing relative to the starting direction. measured in radians
 *
 * param   tPosition         a double array with three indexes recording the position of the target (x,y,z(up))
 *
 *
 *
 * return headingToTarget          the heading that the turret needs to point to (relative to the field) to face the target
 * return pitchToTarget        the pitch that the turret needs to be at to hit the target.
 *
 * how the teleop will handle this
 *
 */
//todo: aimbot doesn't work, there are issues with the pitch to target calculations, and issues with heading to target calculations, goes infinite after slight movements
// no changes have been made in response to this as the issue is assumed to be because of the PositionAndTargetManager
public class AimAssist {
    double turretHeight;

    //the gravitational constant g
    final double g = 9.80655;

    //variables defining the robots characteristics
    double[] robotPosition = new double[2]; // x, y, coords of robot, measured in meters
    double robotHeading;        // direction the robot is facing relative to the field (where pi/2 is toward the wall with the goals), measured in radians

    //variables defining the targets characteristics
    double[] targetPosition = new double[3]; // x, y, z, coords of the robot, measured in meters

    //variables that are calculated
    public double headingToTarget; // the direction that the turret needs to face relative to the wall with the goals (pi/2), measured in radians
    public double pitchToTarget; // the pitch that the turret needs to be at to hit the target, measured in radians

    ////////////////////////////// constructors //////////////////////////////
    /**constructor (tuHeading and tuPitch should both be zero, unless the robot starts in an awkward position**/
    public AimAssist(HardwareUltimateGoal robot, double[] rPosition, double rHeading, double[] tPosition) {
        robotPosition[0] = rPosition[0];
        robotPosition[1] = rPosition[1];

        robotHeading = rHeading;

        targetPosition[0] = tPosition[0];
        targetPosition[1] = tPosition[1];
        targetPosition[2] = tPosition[2];

        turretHeight = robot.turretHeight;
    }

    ////////////////////////////// update method //////////////////////////////
    /**
     * method to update the the variables to the most recent values
     */
    public void update(double[] rPosition, double rHeading, double[] tPosition) {
        robotPosition[0] = rPosition[0];
        robotPosition[1] = rPosition[1];

        robotHeading = rHeading;

        targetPosition[0] = tPosition[0];
        targetPosition[1] = tPosition[1];
        targetPosition[2] = tPosition[2];

        headingToTarget = headingCalculation();
        pitchToTarget = pitchCalculation();
    }

    ////////////////////////////// calculating methods //////////////////////////////
    /* if this returns:
    -1.0; there was an error in the atan2 calculation
     */
    /**
     * the method to calculate the heading that the turret needs to point at in order to point to the target
     * @return headingToTarget the heading to the target relative to the field
     */
    private double headingCalculation () {
        //x and y values of the robot
        double rX = robotPosition[0];
        double rY = robotPosition[1];
        //x and y values of the target
        double tX = targetPosition[0];
        double tY = targetPosition[1];
        //x and y values of imaginary triangle with the hypotenuse being the line between the turret and the target
        double x = tX - rX;
        double y = tY -rY;

        // heading relative to the field (theta = pi/2 , means robot is pointing toward the wall with the goals)
        double heading = Math.atan2(y,x); //calculated in radians, gives an accurate angle in the range (pi , -pi), with 0 being parrallel to the x-axis
        return heading;
    }

    /*
     * calculations for more accurate pitch calculations attempt 1
     *
     * knowns:
     *  y displacement needed = height
     *  mass of ring = 0.065lbs = 0.029483504kg
     *  weight of ring (downward acceleration newtons?) = mass * -9.80665m/s/s = -0.2891344045016N
     *  a of y = -g = -9.80665m/s/s
     *
     *  x displacement needed = distance
     *
     *  knowns to calculate initial velocity?
     *      maximum RPM of the motor (tetrix torqueNADO) (no load) = 100 RPM = 5/3 RPS
     *      stall torque of the motor = 700 oz/in. = 4.94308628333331 newton meter
     *
     *      diameter of the flywheels (andymark 4in compliant wheels) = 4in = 0.1016m
     *      circumference of flywheels = 0.1016 * pi = 0.319185813605m
     *      mass of the flywheels:  [0.228, 0.269] Lbs (between those numbers) = [0.1034190604, 0.12201635] kg ~= 0.1127177052 kg
     *      velocity of flywheel at point of contact? = 5/3 r/s * (0.1016m * pi)/r = (0.508)*pi/3 m/s = 0.169333...*pi m/s = 0.531976 m/s
     *
     *      with NevaRest motors (1780 rpm)
     *      velocity = 29.6666 r/s * (0.1016 * pi)/r = 9.469 m/s
     *
     *
     *      torque applied to the wheel = torque of the motor / 2 (because 1 motor is driving two outputs) = 2.47154314167 newton meter (may be double this, if the torque of the two flywheels add up)
     *      force at point of contact = torque to wheel / radius = 2.47154314167Nm / 0.1016/2m = 48.6524N
     *
     *
     *  need to: find the pitch
     *  assuming that the fly wheels are able to apply all of their speed to the ring, without the ring slipping, basically, assuming high friction
     *  initial velocity of the ring = velocity of the flywheel at point of contact = 0.531976 m/s
     *
     *  know:
     *  velocity = 0.531976 m/s, (if the flywheels are able to transfer all their velocity to the ring)
     *  x displacement = distance,
     *  y displacement = height,
     *  x acceleration = 0m/s/s, (this may be incorrect, depends on how the flywheels accelerate the ring)
     *  y acceleration = -9.8m/s/s
     */

    /*calculations for more accurate pitch calculations attempt 2 (will be mostly done on paper)
     *
     * knowns:
     *  y displacement needed = height = h
     *  mass of ring = 0.065lbs = 0.029483504kg
     *  weight of ring (downward acceleration newtons?) = mass * -9.80665m/s/s = -0.2891344045016N
     *  a of y = -g = -9.80665m/s/s
     *
     *  x displacement needed = distance = d
     *
     *  calculation for angle:
     *  angle = Math.atan( (v*v +- Math.sqrt(v*v*v*v - g * (g*d*d + 2*h*v*v)) )/(g * d) )
     *
     **/
    // TODO 10/14/2020 update calculation of launch speed (magnitude of velocity) by accounting for friction, then using the launch speed determined by Aaron

    /* if this returns:
    -1; required pitch would send ring out of bounds
    -2; there was an error in the calculation, the try catch caught the error
     */
    private double pitchCalculation() {
        //trajectory height and range caps
        final double heightCap = 1.524; //meters (5ft)
        final double rangeCap = 4.572; //meters (15ft)

        //find distance and height
        // calculate distance to target
        double x = targetPosition[0] - robotPosition[0];
        double y = targetPosition[1] - robotPosition[1];
        double d = Math.sqrt( x*x + y*y);
        // calculate height to target
        double h = targetPosition[2] - turretHeight;

        try {
            //the heading angle, what is being calculated
            double angle;
            double a1;
            double a2;
            // the launch speed TODO update this when Aaron finishes the experiment to determine launch speed
            final double v = 5.08;

            a1 = Math.atan( ( (v*v) + Math.sqrt( (v*v*v*v) - (g * (g * (d*d) + (2*h * (v*v)) )) ) ) / (g * d) );//+
            a2 = Math.atan( ( (v*v) - Math.sqrt( (v*v*v*v) - (g * (g * (d*d) + (2*h * (v*v)) )) ) ) / (g * d) );//-

            //decide the optimal angle
            angle = a2;
            if (a1 < a2)
            {
                angle = a1;
            }

            //make sure that the height and length of the trajectory stay within bounds
            // height || range
            if ( (((v*v * Math.sin(angle) * Math.sin(angle)) / (2 * g)) >= (heightCap - turretHeight)) || (((v*v * Math.sin(2 * angle)) / (g)) >= rangeCap) )  return -2.0;

            return angle;
        } catch (Exception e) {
            //return -1
            // TODO 10/14/2020 in teleOP and autonomous programs that use this, add a thing that sends the following message to the phone "out of range, move closer" when -1 is returned
            return -1.0;
        }
    }

    ////////////////////////////////// get methods ////////////////////////////////
    /**
     * method to get the pitch
     * @return pitch returns the pitch the turret need to rotate, measured in radians
     */
    public double getPitchToTarget() {
        return pitchToTarget;
    }
    /**
     * method to get the heading
     * @return heading returns the heading relative to the robot that the turret needs to rotate to
     */
    public double getHeadingToTarget() {
        return headingToTarget;
    }
}
