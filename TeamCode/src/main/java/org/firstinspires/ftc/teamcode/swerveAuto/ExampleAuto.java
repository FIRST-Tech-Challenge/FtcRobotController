package org.firstinspires.ftc.teamcode.swerveAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDriveBase;
import org.firstinspires.ftc.teamcode.teamUtil.*;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.ConfiguredOpMode;
import org.firstinspires.ftc.teamcode.teamUtil.trajectoryAssembly.*;

/**
 * This is outdated lmao
 */
@Disabled
@Autonomous(name = "Swerve Auto Test", group = "Worlds")
public class ExampleAuto extends ConfiguredOpMode {
	SwerveDriveBase swerve = new SwerveDriveBase(RobotConstants.enabledModules.BOTH);
	TrajectoryAssembly testDrive;
	
	@Override
	public void superInit() {
		testDrive = new TrajectoryAssembly()
				.addOffsetActionMarker(10, () -> {
					telemetry.addLine("hey");
				})
				.addSegment(new Pose2D(new Coordinate2D(1000, 0), new Angle(90)))
				.addSegment(new Pose2D(new Coordinate2D(1000, 1000), new Angle(90)))
				.addSegment(new Pose2D(new Coordinate2D(0, 1000), new Angle(90)))
				.addOffsetActionMarker(1, () -> {
					telemetry.addLine("hey");
				})
				.addSegment(new Pose2D(new Coordinate2D(0, 0), new Angle(90)))
				
				.build();
	}
	
	@Override
	public void registerTriggers() {
	}
	
	@Override
	public void superInit_Loop() {
	
	}
	
	@Override
	public void superStart() {
		RobotConfig.currentTrajectoryAssembly = testDrive;
	}
	
	@Override
	public void superLoop() {
		swerve.trajectoryFollowerUpdate();
	}
	
	@Override
	public void superStop() {
	
	}
}