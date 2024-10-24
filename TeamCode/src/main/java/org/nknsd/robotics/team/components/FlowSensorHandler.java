package org.nknsd.robotics.team.components;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.nknsd.robotics.framework.NKNComponent;


//Offset is:
//0.590551
//3.54331
public class FlowSensorHandler implements NKNComponent {

    public static class PoseData {
        public final double heading;
        public final double x;
        public final double y;

        public PoseData(double heading, double x, double y) {
            this.heading = heading;
            this.x = x;
            this.y = y;
        }
    }

    public static class OdometryData {
        public final PoseData pos;
        public final PoseData vel;
        public final PoseData acc;

        public OdometryData(PoseData pos, PoseData vel, PoseData acc) {
            this.pos = pos;
            this.vel = vel;
            this.acc = acc;
        }
    }

    SparkFunOTOS flowSensor;

    String flowName;
    int xOffset;
    int yOffset;
    int hOffset;

    public FlowSensorHandler(String flowName, int xOffset, int yOffset, int hOffset) {
        this.flowName = flowName;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.hOffset = hOffset;
    }

    private void configureOtos(int xOffset, int yOffset, int hOffset) {
        flowSensor.setLinearUnit(DistanceUnit.INCH);
        flowSensor.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. For example, if the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(xOffset, yOffset, hOffset);
        flowSensor.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        flowSensor.setLinearScalar(1.0);
        flowSensor.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        flowSensor.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        flowSensor.resetTracking();
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        flowSensor = hardwareMap.get(SparkFunOTOS.class, flowName);
        configureOtos(xOffset, yOffset, hOffset);
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        String flowString = "[ heading " + getOdometryData().pos.heading + " x " + getOdometryData().pos.x + " y " + getOdometryData().pos.y + "]";
        telemetry.addData("flowSensor", flowString);
    }

    public OdometryData getOdometryData() {
        SparkFunOTOS.Pose2D pos2d = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D vel2d = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D acc2d = new SparkFunOTOS.Pose2D();
        flowSensor.getPosVelAcc(pos2d, vel2d, acc2d);
        PoseData pos = new PoseData(pos2d.h, pos2d.x, pos2d.y);
        PoseData vel = new PoseData(vel2d.h, vel2d.x, vel2d.y);
        PoseData acc = new PoseData(acc2d.h, acc2d.x, acc2d.y);
        return new OdometryData(pos, vel, acc);
    }


}
