package org.firstinspires.ftc.teamcode.teamUtil;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Scheduler;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.TrajectoryAssembly;

import java.util.ArrayList;
import java.util.List;

public class RobotConfig {
    public OpMode opMode;

    private static RobotConfig rInstance;
    public List<LynxModule> allHubs;

    private RobotConfig(OpMode opMode){
        this.opMode = opMode;
        this.opMode.telemetry.setAutoClear(false);
        Scheduler.freshInstance(this);
        allHubs = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public static RobotConfig freshInstance(OpMode opMode){
        rInstance = new RobotConfig(opMode);
        return rInstance;
    }

    public static RobotConfig getInstance(){
        return rInstance;
    }

    public ArrayList<Subsystem> subsystems = new ArrayList<>();
    public static Pose2D
            robotPose2D,
            previousRobotPose2D;
    public static TrajectoryAssembly currentTrajectoryAssembly;
    public static final ElapsedTime elapsedTime = new ElapsedTime();
}