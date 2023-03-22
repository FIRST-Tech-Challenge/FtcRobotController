package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LimitSwitch;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.webcam.Webcam;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.ConfiguredOpMode;
import org.firstinspires.ftc.teamcode.teamUtil.odometry.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name="RightMedium5Auto", group="smellink")
public class RightMedium5Auto extends ConfiguredOpMode {
	private SampleMecanumDrive mecanum;
	private Arm arm;
	private Wrist wrist;
	private Intake intake;
	private LimitSwitch limitSwitch;
	private Lift lift;
	private Webcam webcam;
	TrajectorySequenceStorage sequence;
	
	@Override
	public void superInit() {
		telemetry.addData("status", "init");
		mecanum = new SampleMecanumDrive(hardwareMap);
		arm = new Arm();
		wrist = new Wrist();
		intake = new Intake();
		limitSwitch = new LimitSwitch();
		lift = new Lift();
		sequence = new TrajectorySequenceStorage().rightMedium5(
				r,
				mecanum,
				lift,
				intake,
				arm,
				wrist,
				webcam
		);
		
		telemetry.setAutoClear(true);
		telemetry.addData("status", "fin");
	}
	
	@Override
	public void registerTriggers() {
	
	}
	
	@Override
	public void superInit_Loop() {
	
	}
	
	@Override
	public void superStart() {
		sequence.startFollowSetSequenceAsync();
	}
	
	@Override
	public void superLoop() {
		sequence.followSetSequenceAsync();
	}
	
	@Override
	public void superStop() {
	
	}
}
