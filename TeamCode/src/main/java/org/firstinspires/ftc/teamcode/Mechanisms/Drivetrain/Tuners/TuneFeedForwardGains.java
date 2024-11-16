package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Tuners;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.Battery;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;
import org.firstinspires.ftc.teamcode.Mechanisms.Utils.Planners.MotionProfile;

@Config
@Autonomous(name = "Test Feed Forward", group = "Autonomous")
public class TuneFeedForwardGains extends LinearOpMode{
    // Create drivetrain object
    Drivetrain drivetrain = null;
    // Use FTCDashboard
    FtcDashboard dashboard;
    public static double maxAcceleration = 50.0;
    public static double maxVelocity = 60;
    public static double maxDistance = 84;
    Battery battery;
    @Override
    public void runOpMode() {
        // Set dashboard
        battery = new Battery(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, battery);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        TelemetryPacket packet = new TelemetryPacket();
        ElapsedTime looptime = new ElapsedTime();
        ElapsedTime lapTime = new ElapsedTime();
        MotionProfile motionProfile = new MotionProfile(maxDistance,maxVelocity,maxAcceleration,maxAcceleration,false);
        MotionProfile reverseMotionProfile = new MotionProfile(maxDistance,maxVelocity,maxAcceleration,maxAcceleration,true);
        boolean reverse = false;
        double deltaT = reverseMotionProfile.getTime();
        double velocity = maxVelocity;
        SimpleMatrix speeds = new SimpleMatrix(
                new double[][]{
                        {0},
                        {0},
                        {0}

                }
        );
        SimpleMatrix accelerations = new SimpleMatrix(
                new double[][]{
                        {0},
                        {0},
                        {0}
                }
        );
        telemetry.addData("Robot velocity ", 0);
        telemetry.addData("Target velocity ", 0);
        telemetry.addLine("State " + 0);
        /*packet.fieldOverlay();
        dashboard.sendTelemetryPacket(packet);*/
        telemetry.update();
        waitForStart();
        looptime.reset();
        lapTime.reset();
        while (opModeIsActive()) {
            // Check if reset is necessary


            drivetrain.localize();
            if (reverse) {
                    velocity = reverseMotionProfile.getVelocity(lapTime.seconds());
                    speeds.set(0,0,velocity);
                    accelerations.set(0,0,reverseMotionProfile.getAcceleration(lapTime.seconds()));
                    drivetrain.setWheelSpeedAcceleration(Utils.inverseKinematics(speeds),Utils.inverseKinematics(accelerations));
            } else {

                    velocity = motionProfile.getVelocity(lapTime.seconds());
                    speeds.set(0,0,velocity);
                    accelerations.set(0,0,motionProfile.getAcceleration(lapTime.seconds()));
                    drivetrain.setWheelSpeedAcceleration(Utils.inverseKinematics(speeds),Utils.inverseKinematics(accelerations));

            }
            looptime.reset();
            telemetry.addData("Robot velocity ", drivetrain.state.get(3,0));
            telemetry.addData("Target velocity ",  velocity);
            /*packet.fieldOverlay()
                    .setRotation(drivetrain.state.get(2,0))
                    .setFill("blue")
                    .fillRect(drivetrain.state.get(0,0), drivetrain.state.get(1,0), 15, 15);
            dashboard.sendTelemetryPacket(packet);
            */
            telemetry.update();
            if (lapTime.seconds() > deltaT) {
                reverse = !reverse;
                lapTime.reset();
            }
        }
    }
}
