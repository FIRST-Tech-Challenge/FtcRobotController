package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllables.Location;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.Trajectories.CubicParametric;
import org.firstinspires.ftc.teamcode.Trajectories.Trajectory2D;

import java.util.ArrayList;

/**
 * @author Zack Horton
 * If you're reading this that means you're cool, I like you!
 * I am Zack Horton from team 11536 and I first made this class in 2020 during my ISP (Independent Senior Project)
 * Anyways, the point of this class was to be a jumping off point for teams in the future to use to learn about coding the robot
 * As well as hopefully to be used (at least partly) in comp
 * Something you should know is that as i'm writing this, all locations are measured in inches,
 * this is not necessary though, I just arbitrarially chose inches and it can be changed in the odometry class by changing one value
 * Now, if you know who I am, that's awesome! Reach out to me, I miss you :(
 * If you don't know who I am, that's even cooler! That means this lived on much longer than I expected or that it got shared with some people from another team or something
 * So if you do see this and don't know me, I'd love to hear from you, my instagram is probably still 'zack_horton' give me a follow and DM me about how the season is going
 * Ok, its 1 AM while I'm writing this so I should probably go to bed but before I do I want to start something. If you're working on this, add your name to the author list so other people know how cool you are
 * @version 1.0
 * @since 1.0
 */
public abstract class ReneBase extends LinearOpMode {

    /*
     * Motors for a mecanum drive base
     * FL       FR
     *
     *
     * BL       BR
     */
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;

    //Location object for the class. I'm not sure if this really gets used because the odometry object has a location built in
    private Location ReneLoc = new Location(0, 0, 0);

    //Odometry object for the class. This holds the location of the robot and has methods to calculate the position based on the raw encoder values
    public Odometry odo;

    public ElapsedTime time = new ElapsedTime();

    /**
     * Method to initialize the motors
     * You'll want to use this on every opMode you write with this class because it is necessary for the motors to work
     */
    public void initMotors()
    {
        telemetry.addData("Initializing Motors: ", "Started");
        FL = hardwareMap.dcMotor.get("motorFL");
        FR = hardwareMap.dcMotor.get("motorFR");
        BL = hardwareMap.dcMotor.get("motorBL");
        BR = hardwareMap.dcMotor.get("motorBR");

        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Initializing Motors: ", "Done");
    }

    /**
     * Method to initialize the location object to a certain position
     * Again, I don't think this is necessary because the odometry object has a loaction object built in
     * @param xPosition X-Coordinate
     * @param yPosition Y-Coordinate
     * @param angle Robot angle
     */
    public void initLocation(double xPosition, double yPosition, double angle)
    {
        telemetry.addData("Initializing Location: ", "Started");
        ReneLoc = new Location(xPosition, yPosition, angle);
        telemetry.addData("Initializing Location: ", "Done");
    }

    /**
     * Method to initialize the odometry object by just creating a new object and updating the odometry object and then resetting it so that it is at 0,0,0
     * You'll want to use this in every opMode you write with this class that uses odometry which should probably be all
     */
    public void initOdometry()
    {
        telemetry.addData("Initializing Odometry: ", "Started");
        odo = new Odometry();
        resetOdo();
        updateOdometry();
        resetOdo();
        telemetry.addData("Initializing Odometry: ", "Done");
    }

    /**
     * Method to call the .update method in the odometry class using the raw encoder values
     * You'll want to use this in the main loop of every opMode you write with this class that uses odometry which should probably be all
     */
    public void updateOdometry()
    {
        odo.update(FL.getCurrentPosition(), FR.getCurrentPosition(), -BL.getCurrentPosition(), time.seconds());
    }

    /**
     * Method to drive the robot based on vectors from -1 to 1 with 0 being no movement.
     * This is built for a mecanum base robot but I guess could be used for a holonomic robot too
     * The values are optimized for a controller so if you just plug in the raw controller values that should work
     * It is not quite perfect in that if a motor is moving in one direction and turning, the power just gets clipped to 1 rather than taking that off of another motor
     * With that said, it is what I and I imagine most people are used to driving with
     * @param vertical Forward and backward vector (gamepad?.left_stick_y)
     * @param horizontal Left and right vector (gamepad?.left_stick_x)
     * @param rotational turn vector (gamepad?.right_stick_x)
     */
    public void driveWithVectors(double vertical, double horizontal, double rotational)
    {
        double magnitude = Math.sqrt((horizontal*horizontal) + (vertical*vertical));
        double angle = Math.atan2(horizontal, -vertical) + (Math.PI / 4);

        double FL_BR = Math.sin(angle) * magnitude;
        double FR_BL = Math.cos(angle) * magnitude;

        FL.setPower(FL_BR + rotational);
        FR.setPower(FR_BL - rotational);
        BL.setPower(FR_BL + rotational);
        BR.setPower(FL_BR - rotational);
    }

    /**
     * Method to drive the robot based on vectors from -1 to 1 with 0 being no movement and make sure that no matter the orientation of the robot, the forward and back always coresponds to y movement and same with x.
     * This is built for a mecanum base robot but I guess could be used for a holonomic robot too
     * The values are optimized for a controller so if you just plug in the raw controller values that should work
     * It is not quite perfect in that if a motor is moving in one direction and turning, the power just gets clipped to 1 rather than taking that off of another motor
     * With that said, it is what I and I imagine most people are used to driving with
     * Some people prefer this driving style but not too many, it is mainly used for autonomous-based functions
     * @param vertical Forward and backward vector (gamepad?.left_stick_y)
     * @param horizontal Left and right vector (gamepad?.left_stick_x)
     * @param rotational turn vector (gamepad?.right_stick_x)
     */
    public void driveWithVectorsGlobal(double vertical, double horizontal, double rotational)
    {
        vertical = MathFunctions.clip(vertical, 1, -1);
        horizontal = MathFunctions.clip(horizontal, 1, -1);
        rotational = MathFunctions.clip(rotational, 1, -1);

        double magnitude = Math.sqrt((horizontal*horizontal) + (vertical*vertical));
        double angle = Math.atan2(horizontal, -vertical) + (Math.PI / 4) + odo.getAngle();

        double FL_BR = Math.sin(angle) * magnitude;
        double FR_BL = Math.cos(angle) * magnitude;

        double flPow = MathFunctions.clip(FL_BR + rotational, 1, -1);
        double frPow = MathFunctions.clip(FR_BL - rotational, 1, -1);
        double blPow = MathFunctions.clip(FR_BL + rotational, 1, -1);
        double brPow = MathFunctions.clip(FL_BR - rotational, 1, -1);

        FL.setPower(flPow);
        FR.setPower(frPow);
        BL.setPower(blPow);
        BR.setPower(brPow);
    }

    /**
     * Method to print the current location of the robot to the driver station using telemetry
     * Note: It is in the order (x, y, angle) unlike (y, x, angle) which is usually used in this project
     * Note 2: Definetly use this for debugging and practice but telemetry is notoriously slow and can slow down the robot cycles
     *         which then makes the tracking less accurate so I would stay away from having this in both auto and teleOp loops for competitions
     */
    public void printLocation()
    {
        telemetry.addData("X: ", odo.getX());
        telemetry.addData("Y: ", odo.getY());
        telemetry.addData("A: ", odo.getAngle());
        telemetry.update();
    }

    /**
     * Method to print the raw encoder values to the driver station using telemetry
     * Note: Definetly use this for debugging and practice but telemetry is notoriously slow and can slow down the robot cycles
     *       which then makes the tracking less accurate so I would stay away from having this in both auto and teleOp loops for competitions
     */
    public void printEncoderValues()
    {
        telemetry.addData("L: ", FL.getCurrentPosition());
        telemetry.addData("R: ", FR.getCurrentPosition());
        telemetry.addData("M: ", BL.getCurrentPosition());
        telemetry.update();
    }

    /**
     * Calls the .resetAll method in the odometry class to set the position of the robot to (0,0,0)
     * This does not move the robot, just resets the stored values of the location
     */
    public void resetOdo()
    {
        odo.resetAll();
    }

    /**
     * Method to calculate the next step to get the robot to a certain position
     * This method does not contain a loop and so if it is used once the robot will calculate the current best powers to set the motors to to get to the
     * target position and stay at those powers unless they are set to do something else
     * The robot will drive to the desired location and also try to turn toward it
     * This gives a funny phenomena where when the robot reaches the target, if this method is called again and again, it will make weird turning movements
     * I may edit this functionality but as of right now it is something you have to live with
     * @param x Target x position
     * @param y Target y position
     * @param maxPow Maximum power you would like the motors to get set too. This may get exceeded during turns but shouldn't allow the robot to exceed a certain speed
     */
    public void driveToPointItterative(double x, double y, double maxPow, double proportional)
    {

        double pA = 1;

        double errorX = odo.getX() - x;
        double errorY = odo.getY() - y;
        double errorA = MathFunctions.angleWrap(odo.getAngle() - Math.atan2(errorX, -errorY));

        double xPower = MathFunctions.clip(errorX * -proportional, maxPow, -maxPow);
        double yPower = MathFunctions.clip(errorY * proportional, maxPow, -maxPow);
        double aPower = MathFunctions.clip(errorA * pA, maxPow, -maxPow);

        driveWithVectorsGlobal(yPower, xPower, aPower);
    }

    /**
     * Method to drive to a position using a loop.
     * Uses the driveToPointItterative method to calculate the best powers to use to get there
     * @param x Target x position
     * @param y Target y position
     * @param maxPow Maximum power you would like the motors to get set too. This may get exceeded during turns but shouldn't allow the robot to exceed a certain speed
     * @param allowance Radius the robot must be within the target position for it to exit the loop
     */
    public void driveToPointLoop(double x, double y, double maxPow, double allowance)
    {
        double dist = Math.hypot(odo.getX() - x, odo.getY() - y);

        while ((dist > allowance) && (opModeIsActive()))
        {
            updateOdometry();
            dist = Math.hypot(odo.getX() - x, odo.getY() - y);
            driveToPointItterative(x, y, maxPow, .1);
            telemetry.addData("X : ", odo.getX());
            telemetry.addData("Y : ", odo.getY());
            telemetry.addData("A : ", odo.getAngle());
            telemetry.addData("target : ", "" + x + y);
            telemetry.update();
        }
        driveWithVectors(0,0,0);
    }

    /**
     * Untested method based on Gluten Free's (FTC team 11115) "road runner" code
     * This is not an exact copy but rather and adaptation, the basic idea is the same though
     * Here is the first video in their series explaining their code: https://www.youtube.com/watch?v=3l7ZNJ21wMo
     * I will make something to explain exactly what this does and why I chose to do what it does and put it here:
     * @param path ArrayList of Locations you would like the robot to visit
     * @param circleRadius Radius of the circle around the robot that is used for tracking
     * @param timeout If the time excedes this, the robot will stop ****NOT IMPLEMENTED****
     */
    public void purePursuit(ArrayList<Location> path, double circleRadius, double timeout)
    {
        double endTolerance = 3; //measured in inches

        int locationInPath = 0;

        double startTime = time.seconds();

        if (path.size() < 2){
            telemetry.addData("Not enough target points : ", path.size());
            telemetry.update();
            return;
        }

        Location currentTarget = new Location();

        while((locationInPath < path.size()-1) && opModeIsActive())
        {
            updateOdometry();
            Location startLine = path.get(locationInPath);
            Location endLine = path.get(locationInPath + 1);
            if (startLine.xPos == endLine.xPos)
            {
                endLine.xPos += .00001;
            }

            while(((MathFunctions.distance(endLine, odo.getX(), odo.getY()) > circleRadius) && opModeIsActive()) && (time.seconds() - startTime < timeout))
            {
                updateOdometry();
                currentTarget = MathFunctions.lineCircleIntersection(startLine, endLine, odo.getX(), odo.getY(), circleRadius);
                double errorX = odo.getX() - currentTarget.xPos;
                double errorY = odo.getY() - currentTarget.yPos;
                double errorA = MathFunctions.angleWrap(odo.getAngle() - Math.atan2(errorX, -errorY));
                double h = Math.hypot(errorX, errorY);

                driveWithVectorsGlobal(errorX / h, errorY / h, errorA);
            }

        }

        currentTarget.setLoc(path.get(path.size()-1));

        driveToPointLoop(currentTarget.xPos, currentTarget.yPos, 1, endTolerance);

    }

    /**
     * Untested method to drive straight towards a point using a motion profile made of cubic splines
     * @param target Location which is the target we are looking to go towards
     * @param timeToDrive The amount of time we want it to take for the robot to travel the profile
     * @param maxPow The maximum power we allow to go to the motors, typically 1
     */
    public void MotionProfilingDriveToPoint(Location target, double timeToDrive, double maxPow)
    {
        double startTime = time.seconds();
        CubicParametric parX = new CubicParametric( -2 * target.xPos * Math.pow(timeToDrive, -3),
                                                    3 * target.xPos * Math.pow(timeToDrive, -2),
                                                    0, 0);

        CubicParametric parY = new CubicParametric( -2 * target.yPos * Math.pow(timeToDrive, -3),
                                                    3 * target.yPos * Math.pow(timeToDrive, -2),
                                                    0, 0);

        double kP = .1;
        double kI = 0;
        double kD = 0;
        double kV = -0.06;
        double kA = -.005;
        double currentTimeInLoop = time.seconds()-startTime;

        telemetry.addData("Loop: ", "started");
        telemetry.update();

        while((currentTimeInLoop) < timeToDrive)
        {
            updateOdometry();
            currentTimeInLoop = time.seconds()-startTime;
            double currentTargetX = parX.getVal(currentTimeInLoop);
            double currentTargetY = parY.getVal(currentTimeInLoop);

            double errorX = odo.getX() - currentTargetX;
            double errorY = odo.getY() - currentTargetY;

            double xP = errorX * -kP;
            double yP = errorY * kP;
            double xV = parX.getVelocity(currentTimeInLoop) * -kV;
            double yV = parY.getVelocity(currentTimeInLoop) * kV;
            double xA = parX.getAcceleration(currentTimeInLoop) * -kA;
            double yA = parY.getAcceleration(currentTimeInLoop) * kA;
            //double errorA = MathFunctions.angleWrap(odo.getAngle() - Math.atan2(errorX, -errorY));

            double xPower = MathFunctions.clip(xP + xV + xA, maxPow, -maxPow);
            double yPower = MathFunctions.clip(yP + yV + yA, maxPow, -maxPow);
            //double aPower = MathFunctions.clip(errorA * pA, maxPow, -maxPow);

            driveWithVectorsGlobal(yPower, xPower, 0);
        }
        driveWithVectors(0,0,0);
        telemetry.addData("Loop: ", "ended");
        printLocation();

    }

    public void followMotionProfile2D(Trajectory2D path, double kP, double kV, double kA, double kAngle)
    {
        double startTime = time.seconds();
        double currentTimeInLoop = time.seconds()-startTime;

        while (currentTimeInLoop < path.totalTime)
        {
            updateOdometry();
            currentTimeInLoop = time.seconds()-startTime;

            double xP = (odo.getX() - path.getValX(currentTimeInLoop)) * kP;
            double yP = (odo.getY() - path.getValY(currentTimeInLoop)) * kP;

            double xV = path.getVelocityX(currentTimeInLoop) * kV;
            double yV = path.getVelocityY(currentTimeInLoop) * kV;

            double xA = path.getAccelerationX(currentTimeInLoop) * kA;
            double yA = path.getAccelerationY(currentTimeInLoop) * kA;

            double anglePow = MathFunctions.angleWrap(odo.getAngle() - Math.atan2(odo.getX() - path.getValX(currentTimeInLoop),
                                                                                             -(odo.getY() - path.getValY(currentTimeInLoop)))) * kAngle;

            double xPower = MathFunctions.clip(xP + xV + xA, 1, -1);
            double yPower = MathFunctions.clip(yP + yV + yA, 1, -1);
            double aPower = MathFunctions.clip(anglePow, 1, -1);

            driveWithVectorsGlobal(yPower, xPower, aPower);

        }
    }


}
