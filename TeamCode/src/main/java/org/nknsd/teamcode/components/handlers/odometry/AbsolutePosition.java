package org.nknsd.teamcode.components.handlers.odometry;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.components.sensors.FlowSensor;
import org.nknsd.teamcode.frameworks.NKNComponent;

public class AbsolutePosition implements NKNComponent {

    private final static int SAMPLE_TIME_MS = 25;

    private SparkFunOTOS.Pose2D avPos = new SparkFunOTOS.Pose2D(0, 0, 0);


    private SparkFunOTOS.Pose2D velocity = new SparkFunOTOS.Pose2D();

    final FlowSensor sensor1;
    final FlowSensor sensor2;
    private double lastReadTime;


    public AbsolutePosition(FlowSensor sensor1, FlowSensor sensor2) {
        this.sensor1 = sensor1;
        this.sensor2 = sensor2;
    }
    public AbsolutePosition(FlowSensor sensor1){
        this.sensor1 = sensor1;
        sensor2 = null;
    }

    public SparkFunOTOS.Pose2D getVelocity() {
        return velocity;
    }

    //  averages both sensors
    public SparkFunOTOS.Pose2D getPosition() {
        return avPos;
    }

    /**
     * returns the position of the robot as a double array instead of a SparkFunOTOS.Pose2D
     * @return the respective x, y, and h values of the robot as doubles.
     */
    public double[] getDoublePosition(){
        double[] positions = new double[3];
        positions[0] = avPos.x;
        positions[1] = avPos.y;
        positions[2] = avPos.h;
        return positions;
    }

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
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
        return "Flow Handler";
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {
        if (runtime.milliseconds() - lastReadTime < SAMPLE_TIME_MS)
            return;

        SparkFunOTOS.Pose2D pos1;
        SparkFunOTOS.Pose2D pos2;

        pos1 = sensor1.getPosition();
        if(sensor2 != null) {
            pos2 = sensor2.getPosition();
        } else {
            pos2 = sensor1.getPosition();
        }

        SparkFunOTOS.Pose2D prevPos = avPos;
        avPos = new SparkFunOTOS.Pose2D();

        double hSign = -1;
        if (((Math.abs(pos1.h) + Math.abs(pos2.h)) / 2 > Math.PI / 2) || ((Math.abs(pos1.h) + Math.abs(pos2.h)) / 2 < -Math.PI / 2)) {
            if (Math.abs(pos1.h) < Math.abs(pos2.h)) {
                if (pos1.h > 0) {
                    hSign = 1;
                }
            } else {
                if (pos2.h > 0) {
                    hSign = 1;
                }
            }
            avPos.h = ((Math.abs(pos1.h) + Math.abs(pos2.h)) / 2) * hSign;
        } else {
            avPos.h = (pos1.h + pos2.h) / 2;
        }

//      TODO: Test heading, may be negative
        avPos.h = -avPos.h;
        avPos.x = (pos1.x + pos2.x) / 2;
        avPos.y = (pos1.y + pos2.y) / 2;

        double interval = runtime.milliseconds() - lastReadTime;
        lastReadTime = runtime.milliseconds();

        velocity = new SparkFunOTOS.Pose2D(
                (avPos.x-prevPos.x) / interval,
                (avPos.y-prevPos.y) / interval,
                (avPos.h-prevPos.h) / interval
        );

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("pos", printPose2D(getPosition()));
    }

    static public String printPose2D(SparkFunOTOS.Pose2D pos) {
        return "x: " + String.format("%.2f", pos.x) + " y: " + String.format("%.2f", pos.y) + " h: " + String.format("%.2f", pos.h);
    }
}
