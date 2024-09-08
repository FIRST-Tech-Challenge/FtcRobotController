package org.firstinspires.ftc.teamcode.DriveTrain.Tuners;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.DriveTrain.Drivetrain;

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
                speeds.set(0,0,speeds.get(0,0)+acceleration*looptime.milliseconds());
                speeds.set(1,0,speeds.get(1,0)+acceleration*looptime.milliseconds());
                speeds.set(2,0,speeds.get(2,0)+acceleration*looptime.milliseconds());
                speeds.set(3,0,speeds.get(3,0)+acceleration*looptime.milliseconds());
                drivetrain.setWheelSpeedAcceleration(speeds,accelerations);
                if (lapTime.milliseconds()>t/2){
                    state++;
                    lapTime.reset();
                }
            } else if (state == 1){
                speeds.set(0,0,speeds.get(0,0));
                speeds.set(1,0,speeds.get(1,0));
                speeds.set(2,0,speeds.get(2,0));
                speeds.set(3,0,speeds.get(3,0));
                drivetrain.setWheelSpeedAcceleration(speeds,accelerations);
                if (lapTime.milliseconds()>t){
                    state++;
                    lapTime.reset();
                }
            }
            else if (state == 2){
                speeds.set(0,0,speeds.get(0,0)-acceleration*looptime.milliseconds());
                speeds.set(1,0,speeds.get(1,0)-acceleration*looptime.milliseconds());
                speeds.set(2,0,speeds.get(2,0)-acceleration*looptime.milliseconds());
                speeds.set(3,0,speeds.get(3,0)-acceleration*looptime.milliseconds());
                drivetrain.setWheelSpeedAcceleration(speeds,accelerations);
                if (lapTime.milliseconds()>t){
                    state++;
                    lapTime.reset();
                }
            }  else if (state == 3){
                speeds.set(0,0,speeds.get(0,0));
                speeds.set(1,0,speeds.get(1,0));
                speeds.set(2,0,speeds.get(2,0));
                speeds.set(3,0,speeds.get(3,0));
                drivetrain.setWheelSpeedAcceleration(speeds,accelerations);
                if (lapTime.milliseconds()>t){
                    state++;
                    lapTime.reset();
                }
            }
            else if (state == 4) {
                speeds.set(0, 0, speeds.get(0, 0) -acceleration*looptime.milliseconds());
                speeds.set(1, 0, speeds.get(1, 0) -acceleration*looptime.milliseconds());
                speeds.set(2, 0, speeds.get(2, 0) -acceleration*looptime.milliseconds());
                speeds.set(3, 0, speeds.get(3, 0)-acceleration*looptime.milliseconds());
                drivetrain.setWheelSpeedAcceleration(speeds, accelerations);
                    if (lapTime.milliseconds() > t) {
                        state = 1;
                        lapTime.reset();
                    }
                    lapTime.reset();
            }
            looptime.reset();

        }
    }
}
