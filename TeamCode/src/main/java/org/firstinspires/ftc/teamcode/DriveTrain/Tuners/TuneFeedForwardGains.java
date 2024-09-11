package org.firstinspires.ftc.teamcode.DriveTrain.Tuners;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.DriveTrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Controllers.FeedForward;

@Config
@Autonomous(name = "Test Feed Forward", group = "Autonomous")
public class TuneFeedForwardGains extends LinearOpMode{
    // Create drivetrain object
    Drivetrain drivetrain = null;
    // Use FTCDashboard
    FtcDashboard dashboard;
    double state=0;
    public static double acceleration = 0;
    public static double t = 2000;
    @Override
    public void runOpMode() {
        // Set dashboard
        drivetrain = new Drivetrain(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        ElapsedTime looptime = new ElapsedTime();
        ElapsedTime lapTime = new ElapsedTime();
        SimpleMatrix speeds = new SimpleMatrix(
                new double[][]{
                        {1},
                        {1},
                        {1},
                        {1}

                }
        );
        SimpleMatrix accelerations = new SimpleMatrix(
                new double[][]{
                        {0},
                        {0},
                        {0},
                        {0}
                }
        );
        waitForStart();
        looptime.reset();
        while (opModeIsActive()) {
            drivetrain.localize();
            if (state == 0){
                drivetrain.setWheelSpeedAcceleration(speeds,accelerations);
                if (lapTime.milliseconds()>t/2){
                    state++;
                    lapTime.reset();
                }
            } else if (state == 1){
                drivetrain.setWheelSpeedAcceleration(speeds,accelerations);
                if (lapTime.milliseconds()>t){
                    state++;
                    lapTime.reset();
                }
            }
            else if (state == 2){
                drivetrain.setWheelSpeedAcceleration(speeds,accelerations);
                if (lapTime.milliseconds()>t){
                    state++;
                    lapTime.reset();
                }
            }  else if (state == 3){
                drivetrain.setWheelSpeedAcceleration(speeds,accelerations);
                if (lapTime.milliseconds()>t){
                    state++;
                    lapTime.reset();
                }
            }
            else if (state == 4) {
                drivetrain.setWheelSpeedAcceleration(speeds, accelerations);
                    if (lapTime.milliseconds() > t) {
                        state = 1;
                        lapTime.reset();
                    }
            }
            looptime.reset();
        }
    }
}
