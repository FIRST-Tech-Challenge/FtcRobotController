 package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.broombots.sweep.Classes.LocalizationPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class GobildaPinpointModuleFirmware {

    // variable location for stored pinpoint from hardware map
    private final GoBildaPinpointDriver pinpoint;
    private double lastRobotX, lastRobotY;

//    stored x and y and angle components
    private double robotX, robotY; // in inches
    private double robotAngle; // in degrees
    // These are the current velocities of the robot, updates each iteration.
    private double velocityX, velocityY;
    private double time;
    private Telemetry telemetry;
    private ElapsedTime runtime;
    /**
     * Constructor for this class.
     * @param hardwareMap used to get the module from hardware map
     * @param xPodOffset configures the pinpoint for how we have set it up to ensure current results -CM
     * @param yPodOffset configures the pinpoint for how we have set it up to ensure current results -CM
     * @param reset should we reset and recalibrate the sensor and its positions / angles? false for transition between auto and teleop to avoid transition error (non-fatal, just difference of value)
     */
    public GobildaPinpointModuleFirmware(HardwareMap hardwareMap, Telemetry telemetry, double xPodOffset, double yPodOffset, boolean reset){

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(xPodOffset,yPodOffset,DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        if (reset) {
            pinpoint.resetPosAndIMU();
            pinpoint.recalibrateIMU();
        }

        this.telemetry = telemetry;

        runtime = new ElapsedTime();
    }

    /**
     * gets all new position values and sets the values to the corresponding variables.
     * Call this every iteration to ensure non-stale values (stale = old from a different moment in time)
     */
    public void updateOdometry(){
        // store the last iterations values so we can calculate the velocities!
        lastRobotX= robotX;
        lastRobotY = robotY;

        pinpoint.update();
        Pose2D position =  pinpoint.getPosition();
//        Our frame of reference and the gobilda frame of reference is different, so you units needed to change.
        robotX = position.getX(DistanceUnit.INCH);
        robotY = position.getY(DistanceUnit.INCH);
        robotAngle = pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);

        updateVelocity();

    }
    // Updates the velocityX and velocityY variables based new robot variables. MUST CALL WITHIN updateOdometry()!
    private void updateVelocity(){
        double dt = getDeltaTime();
        velocityX = (robotX - lastRobotX)/dt;
        velocityY = (robotY - lastRobotY)/dt;
        telemetry.addData("velocityX", velocityX);
        telemetry.addData("velocityY", velocityY);
    }
    // returns the time between loop iterations - resets between calls!
    private double getDeltaTime(){
        time = runtime.seconds();
        runtime.reset();
        return time;
    }

    // getter for robot X - Inches from center of field, facing from red alliance station
    public double getRobotX(){
        return robotX;
    }
    // getter for robot Y - Inches from center of field, facing from red alliance station
    public double getRobotY(){
        return robotY;
    }
    // getter for robot angle - Degrees from center of field, facing from red alliance station is 0. CounterClockwise is positive rotation
    public double getRobotAngle(){
        return robotAngle;
    }
    // getter for the velocityX variable
    public double getVelocityX(){
        return velocityX;
    }
    // getter for the velocityY variable
    public double getVelocityY(){
        return velocityY;
    }

    /**
     * Override the position of the robot based on external localisation values
     * @param x inches
     * @param y inches
     * @param angle degrees
     */
    public void overridePosition(double x, double y, double angle){
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,x,y, AngleUnit.DEGREES,angle));
        updateOdometry();
    }

    // just reset the IMU value to 0, but don't change anything else or recalibrate
    public void resetIMU(){
        updateOdometry();
        overridePosition(robotX,robotY,0);
    }

    // reset all values
    public void resetPositionAndIMU(){
        pinpoint.resetPosAndIMU();
    }

    // takes around 0.25 seconds, used a quick sample of time stationary to tune the sensor to be more stable
    public void recalibrateIMU(){
        pinpoint.recalibrateIMU();
    }

    public LocalizationPacket getLocalizationPacket(){
        return new LocalizationPacket(getRobotX(),getRobotY(),getRobotAngle(),getVelocityX(),getVelocityY());
    }
}

