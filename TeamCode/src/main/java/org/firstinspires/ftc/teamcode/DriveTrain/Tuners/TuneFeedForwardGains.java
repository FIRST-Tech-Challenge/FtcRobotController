package org.firstinspires.ftc.teamcode.DriveTrain.Tuners;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    public static double maxAcceleration = 75.0;
    public static double t = 1500;
    public static double maxVelocity = 30;
    @Override
    public void runOpMode() {
        // Set dashboard
        drivetrain = new Drivetrain(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        ElapsedTime looptime = new ElapsedTime();
        ElapsedTime lapTime = new ElapsedTime();
        double state=0;
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
        lapTime.reset();
        while (opModeIsActive()) {
            drivetrain.localize();
            if (state == 0){
                aK = maxAcceleration;
                vK = lastVk+aK*looptime.seconds();
                vK = Range.clip(vK, -maxVelocity,maxVelocity);
                lastVk = vK;
                speeds.set(0,0,vK);
                accelerations.set(0,0,aK);
                drivetrain.setWheelSpeedAcceleration(Utils.inverseKinematics(speeds),Utils.inverseKinematics(accelerations));
                if (lapTime.milliseconds()>t/2){
                    state++;
                    lapTime.reset();
                }
            } else if (state == 1){
                aK = 0;
                vK = lastVk+aK*looptime.seconds();
                vK = Range.clip(vK, -maxVelocity,maxVelocity);
                lastVk=vK;
                speeds.set(0,0,vK);
                accelerations.set(0,0,aK);
                drivetrain.setWheelSpeedAcceleration(Utils.inverseKinematics(speeds),Utils.inverseKinematics(accelerations));
                if (lapTime.milliseconds()>t/2){
                    state++;
                    lapTime.reset();
                }
            }
            else if (state == 2){
                aK = -maxAcceleration;
                vK = lastVk+aK*looptime.seconds();
                vK = Range.clip(vK, -maxVelocity,maxVelocity);
                lastVk = vK;
                speeds.set(0,0,vK);
                accelerations.set(0,0,aK);
                drivetrain.setWheelSpeedAcceleration(Utils.inverseKinematics(speeds),Utils.inverseKinematics(accelerations));
                if (lapTime.milliseconds()>t){
                    state++;
                    lapTime.reset();
                }
            }  else if (state == 3){
                aK = 0;
                vK = lastVk+aK*looptime.seconds();
                vK = Range.clip(vK, -maxVelocity,maxVelocity);
                lastVk = vK;
                speeds.set(0,0,vK);
                accelerations.set(0,0,aK);
                drivetrain.setWheelSpeedAcceleration(Utils.inverseKinematics(speeds),Utils.inverseKinematics(accelerations));
                if (lapTime.milliseconds()>t/2){
                    state++;
                    lapTime.reset();
                }
            }
            else if (state == 4) {
                aK = maxAcceleration;
                vK = lastVk+aK*looptime.seconds();
                vK = Range.clip(vK, -maxVelocity,maxVelocity);
                lastVk = vK;
                speeds.set(0,0,vK);
                accelerations.set(0,0,aK);
                drivetrain.setWheelSpeedAcceleration(Utils.inverseKinematics(speeds),Utils.inverseKinematics(accelerations));
                if (lapTime.milliseconds()>t){
                    state=1;
                    lapTime.reset();
                }
            }
            looptime.reset();
            telemetry.addData("Robot velocity ", drivetrain.state.get(3,0));
            telemetry.addData("Target velocity ", vK);
            telemetry.addLine("State " + state);
            telemetry.update();
        }
    }
}
