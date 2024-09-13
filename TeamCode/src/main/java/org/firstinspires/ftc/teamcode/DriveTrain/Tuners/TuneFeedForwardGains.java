package org.firstinspires.ftc.teamcode.DriveTrain.Tuners;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.DriveTrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Controllers.FeedForward;
import org.firstinspires.ftc.teamcode.Utils.Utils;

@Config
@Autonomous(name = "Test Feed Forward", group = "Autonomous")
public class TuneFeedForwardGains extends LinearOpMode{
    // Create drivetrain object
    Drivetrain drivetrain = null;
    // Use FTCDashboard
    FtcDashboard dashboard;
    double state=0;
    public static double maxAcceleration = 0;
    public static double t = 2000;
    @Override
    public void runOpMode() {
        // Set dashboard
        drivetrain = new Drivetrain(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        ElapsedTime looptime = new ElapsedTime();
        ElapsedTime lapTime = new ElapsedTime();
        double vK = 0;
        double lastVk = 0;
        double aK = 0;
        SimpleMatrix speeds = new SimpleMatrix(
                new double[][]{
                        {vK},
                        {0},
                        {0}

                }
        );
        SimpleMatrix accelerations = new SimpleMatrix(
                new double[][]{
                        {aK},
                        {0},
                        {0}
                }
        );
        waitForStart();
        looptime.reset();
        while (opModeIsActive()) {
            drivetrain.localize();
            if (state == 0){
                vK = lastVk*maxAcceleration*lapTime.milliseconds();
                lastVk = vK;
                drivetrain.setWheelSpeedAcceleration(Utils.inverseKinematics(speeds),accelerations);
                if (lapTime.milliseconds()>t/2){
                    state++;
                    lapTime.reset();
                }
            } else if (state == 1){
                drivetrain.setWheelSpeedAcceleration(Utils.inverseKinematics(speeds),accelerations);
                if (lapTime.milliseconds()>t/2){
                    state++;
                    lapTime.reset();
                }
            }
            else if (state == 2){
                vK = lastVk*-maxAcceleration*lapTime.milliseconds();
                lastVk = vK;
                drivetrain.setWheelSpeedAcceleration(Utils.inverseKinematics(speeds),accelerations);
                if (lapTime.milliseconds()>t){
                    state++;
                    lapTime.reset();
                }
            }  else if (state == 3){
                vK = lastVk*lapTime.milliseconds();
                lastVk = vK;
                drivetrain.setWheelSpeedAcceleration(Utils.inverseKinematics(speeds),accelerations);
                if (lapTime.milliseconds()>t/2){
                    state++;
                    lapTime.reset();
                }
            }
            else if (state == 4) {
                vK = lastVk*maxAcceleration*lapTime.milliseconds();
                lastVk = vK;
                drivetrain.setWheelSpeedAcceleration(Utils.inverseKinematics(speeds),accelerations);
                if (lapTime.milliseconds()>t/2){
                    state++;
                    lapTime.reset();
                }
            }
            looptime.reset();
        }
    }
}
