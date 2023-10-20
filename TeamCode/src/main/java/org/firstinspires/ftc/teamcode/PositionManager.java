package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/** Incorporates estimates from multiple sources to create a single positioning estimate
 */
public class PositionManager {
    //Stores the best guess of the robot's position (location + orientation) at any given time. To be accessed by nav methods
    public Position position;
    private Position encoderDelta = new Position();
    public EncoderPositioning encoderPositioning;
    public IMUPositioning imuPositioning;

    private Telemetry telemetry; //This is a way of sending data to the driver controls

    /**
     **Creates a new position manager for the robot starting at (0, 0)**
     *
     * @param hardwareMap the map from the robot where it is reading the values from the sensors
     * @param telemetry the axis point of the robot
     */
    PositionManager(HardwareMap hardwareMap, Telemetry telemetry){
        position = new Position();
        this.telemetry = telemetry;
        initSensors(hardwareMap);
    }

    /**
     **Creates a new position manager for the robot with a defined starting position**
     *
     * @param hardwareMap the map from the robot where it is reading the values from the sensors
     * @param telemetry the axis point of the robot
     * @param x the X position on the robot
     * @param y the Y position on the robot
     * @param theta the rotation of the robot (the angle of the robot, more or less)
     */
    PositionManager(HardwareMap hardwareMap, Telemetry telemetry, double x, double y, double theta){
        position = new Position(x, y, theta, "");
        this.telemetry = telemetry;
        initSensors(hardwareMap);
    }

    private void initSensors(HardwareMap hardwareMap){
        encoderPositioning = new EncoderPositioning();
        imuPositioning = new IMUPositioning(hardwareMap);
    }



}
class EncoderPositioning{
    //NOTE: DO NOT EDIT THE MAGICAL FACTORS UNLESS IF IT IS DEEMED NECESSARY.
    static int ENCODER_COUNTS_PER_ROTATION = 560;
    static double MAGICAL_FACTOR_X = 17.6;
    static double MAGICAL_FACTOR_Y = 18.4;

    static double MAGICAL_RATIO_X = MAGICAL_FACTOR_X / ENCODER_COUNTS_PER_ROTATION;
    static double MAGICAL_RATIO_Y = MAGICAL_FACTOR_Y / ENCODER_COUNTS_PER_ROTATION;

    //Stores the last known encoder values for each wheel. They all start at a value of 0.
    static HashMap<Robot.MotorConfigs, Integer> wheelEncoderCounts = new HashMap<Robot.MotorConfigs, Integer>() {{
        put(Robot.MotorConfigs.FRONT_RIGHT, 0);
        put(Robot.MotorConfigs.FRONT_LEFT, 0);
        put(Robot.MotorConfigs.REAR_RIGHT, 0);
        put(Robot.MotorConfigs.REAR_LEFT, 0);
    }};

    //Stores the encoder counts of each wheel when the last time the robot was at rest.
//    static HashMap<Robot.MotorConfigs, Integer> wheelEncoderCountsAtRest = new HashMap<Robot.MotorConfigs, Integer>() {{
//        put(Robot.MotorConfigs.FRONT_RIGHT, 0);
//        put(Robot.MotorConfigs.FRONT_LEFT, 0);
//        put(Robot.MotorConfigs.REAR_RIGHT, 0);
//        put(Robot.MotorConfigs.REAR_LEFT, 0);
//    }};

    //Stores the angle of the rollers on each Mecanum wheels (values may need to get changed) (USING THE UNIT CIRCLE)
    //NOTE: The d means double, the code automatically reads it as a double, don't ask why it's in there. It is not necessary.
    static HashMap<Robot.MotorConfigs, Double> RollerAngles = new HashMap<Robot.MotorConfigs, Double>() {{
        put(Robot.MotorConfigs.FRONT_RIGHT, Math.PI / 4.d);
        put(Robot.MotorConfigs.FRONT_LEFT, 3 * Math.PI / 4.d );
        put(Robot.MotorConfigs.REAR_RIGHT, Math.PI / 4.d);
        put(Robot.MotorConfigs.REAR_LEFT, 3 * Math.PI / 4.d);
    }};

    /**Checks the robot's encoders to get an estimate of the distance and direction traveled.
     *  Read encoder values and use them to create a Position that represents the robot's movement relative to the last time the encoders were read.
     *  Reset encoder values to zero.
     * @param robot a reference to the robot used for accessing hardware
     * @return a position in the form of a vector from the origin that can be added to an existing measurement
     */
    public Position getDeltaEstimate(Robot robot) {
        updateEncoderCounts(robot); //Takes the robot, reads the values, and stores what it is in the HashMap as wheelEncoderCounts
        double theta = robot.positionManager.position.getRotation();
        double deltaPSumX = 0.0d, deltaPSumY = 0.0d;

        for (HashMap.Entry<Robot.MotorConfigs, Double> rollerAngle : RollerAngles.entrySet()) {
            int encoderCounts = Objects.requireNonNull(wheelEncoderCounts.get(rollerAngle.getKey()));
            double force = rollerAngle.getValue();

            //Updates position count of where the Robot is located on the field. Uses this math equation to simulate where the Robot is on the field.
            if (rollerAngle.getKey().toString().contains("LEFT")) { //Left wheels
                deltaPSumY += -encoderCounts * (Math.cos(theta) * Math.cos(force) - Math.sin(theta) * Math.sin(force)) / 2.0;
                deltaPSumX += -encoderCounts * (Math.cos(theta) * Math.cos(force) + Math.sin(theta) * Math.sin(force)) / 2.0;
            } else { //Right wheels
                deltaPSumY += encoderCounts * (Math.cos(theta) * Math.cos(force) - Math.sin(theta) * Math.sin(force)) / 2.0;
                deltaPSumX += encoderCounts * (Math.cos(theta) * Math.cos(force) + Math.sin(theta) * Math.sin(force)) / 2.0;
            }

        }
        setEncoders(robot);
        return new Position(MAGICAL_RATIO_X * deltaPSumX, MAGICAL_RATIO_Y * deltaPSumY, 0.0, "");
    }

    /**
     ** setEncoders reset the robots X and Y position of the wheels
     * TODO Work with Oscar on getting the code parameters are variables connected through the seperate files.
     * @param robot a reference to the robot used for accessing hardware
     */
    private void setEncoders(Robot robot){
        //update the value of the stored previous value
        wheelEncoderCounts.put(Robot.MotorConfigs.FRONT_RIGHT,robot.frontLeft.getCurrentPosition());
        wheelEncoderCounts.put(Robot.MotorConfigs.FRONT_LEFT, robot.frontRight.getCurrentPosition());
        wheelEncoderCounts.put(Robot.MotorConfigs.REAR_RIGHT, robot.rearLeft.getCurrentPosition());
        wheelEncoderCounts.put(Robot.MotorConfigs.REAR_LEFT,  robot.rearRight.getCurrentPosition());
    }


    private void updateEncoderCounts(Robot robot){
        //get the change in encoder value since the last time the encoders were checked.
        wheelEncoderCounts.put(Robot.MotorConfigs.FRONT_RIGHT, robot.frontLeft.getCurrentPosition() - wheelEncoderCounts.get(Robot.MotorConfigs.FRONT_RIGHT));
        wheelEncoderCounts.put(Robot.MotorConfigs.FRONT_LEFT,  robot.frontRight.getCurrentPosition() - wheelEncoderCounts.get(Robot.MotorConfigs.FRONT_LEFT));
        wheelEncoderCounts.put(Robot.MotorConfigs.REAR_RIGHT,  robot.rearLeft.getCurrentPosition() - wheelEncoderCounts.get(Robot.MotorConfigs.REAR_RIGHT));
        wheelEncoderCounts.put(Robot.MotorConfigs.REAR_LEFT,   robot.rearRight.getCurrentPosition() - wheelEncoderCounts.get(Robot.MotorConfigs.REAR_LEFT));

    }
}


class IMUPositioning{
    static private BNO055IMU imu;
    static private BNO055IMU.Parameters parameters;

    /**
     *
     * @param hardwareMap the direct access to the robot hardware to access the IMU.
     */
    IMUPositioning(HardwareMap hardwareMap){
        //configure the parameters for the IMU
        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false; //we hate logging stuff, so we kept it false :)

        //get the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");

    }

    /** applies the IMU parameters and calibrates hte gyro
     */
    public static void Initialize(){
        imu.initialize(parameters);

        //wait for the gyro to be calibrated
        while (!imu.isGyroCalibrated()){
        }
    }

    /**
     *returns the angle of the robot. (angle is theta)
     */
    public double getAngle(){ //getAngle is theta.
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    /**updates the current rotation of the robot from the IMU
     * @param robot a reference to the robot used for accessing hardware
     */
    void updateRobotOrientation(Robot robot){
        robot.positionManager.position.setRotation(getAngle());
    }
}

//Programmer for PositionManager.java: Tyler M
//ALUMNI HELP: Stephen D.