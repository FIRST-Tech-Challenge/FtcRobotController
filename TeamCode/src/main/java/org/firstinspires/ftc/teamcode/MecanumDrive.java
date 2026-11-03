package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.HardwareMap;

@SuppressWarnings("unused")
public class MecanumDrive {

    private final double Kp = 0.0022; // bigger the error the faster we will fix it
    private final double Ki = 0.00013; // provides extra boost when you get close to the target
    private final double Kd = 0.00015; // dampens overshoot
    private final double WINDUP_LIMIT = 100; // caps integral sum to prevent windup
    private final double closeEnoughXY = 10; // in mm, prev 1.5
    private final double closeEnoughH = .05; // in radians, prev = 0.001
    private double integralSum = 0;
    private double lastError = 0;
    
    private MotorPowerController frontLeft;
    private MotorPowerController frontRight;
    private MotorPowerController backLeft;
    private MotorPowerController backRight;
    private Localizer loc;

     /**
     * Creates an object that controls four standard GoBilda mecanum wheels and 
     * yellowjacket gearmotors. The four motors must have these names in the 
     * Driver Console hardware configureation:
     * frontLeft, frontRight, backLeft, backRight. 
     * To use the field coordinate system for driving, the localizer method must
     * be a valid Locaizer object.
     * 
     * @param hm the hardware map inherited from OpMode (should be passed as simply 'hardwareMap') 
     * @param loc the localizer used to report position and heading. Can be null. 
     * If null, setPowerVector will use the local robot coordinate system where +X is robot nose.
     * moveTo will thorw an error if called with a null localizer, since field position
     * cannot be determined without a localizer
     */
    public MecanumDrive(HardwareMap hm, Localizer loc){
        frontLeft = new MotorPowerController(hm, "frontLeft");
        frontRight = new MotorPowerController(hm, "frontRight");
        backLeft = new MotorPowerController(hm, "backLeft");
        backRight = new MotorPowerController(hm, "backRight");
         
        frontLeft.setDirection(-1);
        frontRight.setDirection(1);
        backLeft.setDirection(-1);
        backRight.setDirection(1);

        this.loc = loc;
    }

    /**
     * Moves the robot with the specified power level in x, y, and rotation
     * Power level is equivalent to speed in most cases, so this provides good manual control using a joystick
     * The robot will move in its own local coordinate system
     * @param fieldX the power level to be applied in the robot X direction (fwd-back) from 1.0 to -1.0
     * @param fieldY the power level to be applied in the robot Y direction (left-right) from 1.0 to -1.0
     * @param rot the power level to be applied to counterclockwise rotation from -1.0 to 1.0
     */
    public void movePOV(double robotX, double robotY, double rot){
        setPowerVectors(robotX, robotY, rot);
    }

    /**
     * Moves the robot with the specified power level in x, y, and rotation
     * Power level is equivalent to speed in most cases, so this provides good manual control using a joystick
     * The robot will move in the field coordinate system if a localizer is defined
     * If no localizer is defined, then the robot will move using its own coordinate system
     * @param fieldX the power level to be applied in the field +X direection from -1.0 to 1.0
     * @param fieldY the power level to be applied in the field +Y direection from -1.0 to 1.0
     * @param rot the power level to be applied to counterclockwise rotation from -1.0 to 1.0
     */
    public void move(double fieldX, double fieldY, double rot){
        // first assume no availabe localization to field coordinate system
        Pose2D pose = null;
        double heading = 0;
        // then if available, find out which way robot is heading in field coordinate system
        if (loc != null){
            pose = loc.getPosition();
            heading = pose.getHeading(AngleUnit.RADIANS);
        }
        
        // transform coordintes from commanded field coords to robot coords
        double cosTheta = Math.cos(heading);
        double sinTheta = Math.sin(heading);
        double robotX = fieldX * cosTheta + fieldY * sinTheta;
        double robotY = fieldX * -sinTheta + fieldY * cosTheta;

        setPowerVectors(robotX, robotY, rot);
    }

    /**
     * Moves the robot toward the specified target point and heading in field coordinates
     * This method is meant to be called rapidly in a loop so that progress toward the target
     * is continually monitored and adjusted
     * @param targetX the targeted field X position in mm
     * @param targetY the targeted field Y position in mm
     * @param targetHeading the targeted heading in radians CCW from the +x axis
     */
    public boolean moveTo(double targetX, double targetY, double targetHeading) {
        // get current position & heading - throws error if no localizer
        loc.update();
        Pose2D currentPosition = loc.getPosition();
        double currentX = currentPosition.getX(DistanceUnit.MM);
        double currentY = currentPosition.getY(DistanceUnit.MM);
        double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);

         // find errors between current and target positions
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double deltaHeading = targetHeading - currentHeading;

         // ignore minor errors to prevent hunting behavior
        if (Math.abs(deltaX) < closeEnoughXY) {
            deltaX = 0;
        }
        if (Math.abs(deltaY) < closeEnoughXY) {
            deltaY = 0;
        }
        if (Math.abs(deltaHeading) < closeEnoughH) {
            deltaHeading = 0;
        }
        if((deltaX == 0) && (deltaY == 0) && (deltaHeading == 0)){
            stop();
            return true;
        }

        // compute PID power levels
        double xPower = pid(deltaX);
        double yPower = pid(deltaY);
        double turnPower = 1.0 * deltaHeading;

        // transform coordintes from commanded field coords to robot coords
        double cosAngle = Math.cos(currentHeading);
        double sinAngle = Math.sin(currentHeading);
        double robotX = xPower * cosAngle + yPower * sinAngle;
        double robotY = xPower * -sinAngle + yPower * cosAngle;

        setPowerVectors(robotX, robotY, turnPower);
        return false;
    }

     private void setPowerVectors(double fwd, double strafe, double rot){
        // see Brogan Pratt's derivation on YouTube (with sign changes for left=+strafe and ccw=+rot)
        frontLeft.setPower( fwd - strafe - rot);
        frontRight.setPower(fwd + strafe + rot);
        backLeft.setPower(  fwd + strafe - rot);
        backRight.setPower( fwd - strafe + rot);
    }

    public void stop(){
        setPowerVectors(0,0,0);
    }

     public double pid(double error){
        double out = Kp * error;
        // TODO add integral and derivative control
        //double derivative = 0;
        //double error = 0;

        //error = delta;

        // rate of change of the error
        //derivative = (delta - lastError) / timer.seconds();

        // sum of all error over time
        //out = (Kp * delta) + (Kd * derivative);

        //this.lastError = delta;

        return out;
    }
}