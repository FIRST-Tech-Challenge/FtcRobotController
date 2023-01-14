package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetectionDeposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "RedTerminalCycleAuto")
@Config
public class OfficialAutoOne extends PowerPlay_AprilTagDetectionDeposit {

	private Arm armControl;
	private Slide slideControl;
	private Claw clawControl;

	public static double xFirstLinear = 59;
	public static double yFirstLinear = -0.75;
	public static double angle = 125;
	public static double xSecondToJunction = 53.5;
	public static double ySecondToJunction = -4;
	public static double xThirdToJunction = 53.5;
	public static double yThirdToJunction = -4;
	public static double xFourthToJunction = 53.5;
	public static double yFourthToJunction = -4;



	public void initialize(){
		armControl = new Arm(hardwareMap);
		slideControl = new Slide(hardwareMap);
		clawControl = new Claw(hardwareMap);
	}

    public void scan(){
        super.runOpMode();
    }

	@Override
	public void runOpMode()
	{
		Pose2d startPose = new Pose2d(0,0,Math.toRadians(180));
		SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
		bot.setPoseEstimate(startPose);

		//Trajectories
		initialize();


		TrajectorySequence junction = bot.trajectorySequenceBuilder(startPose)
				//If 13.53 V, y = -3.5

				//High Junction with Preload
				.UNSTABLE_addTemporalMarkerOffset(0.65,()->{
					slideControl.setHighJunction(telemetry);
					armControl.setExtake();
					clawControl.toggleWristRotate();
					//slideControl.setIntakeOrGround();
					//armControl.setIntake();
				})
				.lineToLinearHeading(new Pose2d(xFirstLinear, yFirstLinear, Math.toRadians(90))) //preload
				//moves down after flipping
				.waitSeconds(0.5)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1450);
				})
				.waitSeconds(.25)
				//opens claw
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				/*
				.waitSeconds(0.15)
				//moves to
				.UNSTABLE_addTemporalMarkerOffset(0.25,()->{
					clawControl.toggleWristRotate();
					slideControl.setCustom(700);
					armControl.setIntake();//ppcocaine
				})
				.waitSeconds(0.25)
				.strafeLeft(14)
				.lineToLinearHeading(new Pose2d(47 ,25.5, Math.toRadians(91.45)))
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1370);
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0.7,()->{
					slideControl.setHighJunction(telemetry);
					armControl.setCustom(990);
					clawControl.toggleWristRotate();
				})
				.lineToLinearHeading(new Pose2d(xSecondToJunction,ySecondToJunction,Math.toRadians(angle)))

				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1450);
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})

				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0.25,()->{

					clawControl.toggleWristRotate();
					slideControl.setCustom(620);
					armControl.setIntake();//ppcocaine
				})
				.waitSeconds(0.25)
				.lineToLinearHeading(new Pose2d(47 ,25.5, Math.toRadians(91.45)))
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1310);
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0.7,()->{
					slideControl.setHighJunction(telemetry);
					armControl.setCustom(990);
					clawControl.toggleWristRotate();
				})
				.lineToLinearHeading(new Pose2d(xThirdToJunction,yThirdToJunction,Math.toRadians(angle)))

				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1450);
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				/*
				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0.25,()->{

					clawControl.toggleWristRotate();
					slideControl.setCustom(640);
					armControl.setIntake();//ppcocaine
				})
				.waitSeconds(0.25)
				.lineToLinearHeading(new Pose2d(47 ,25.5, Math.toRadians(91.45)))
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1310);
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0.65,()->{
					slideControl.setHighJunction(telemetry);
					armControl.setCustom(990);
					clawControl.toggleWristRotate();
				})
				.lineToLinearHeading(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(angle)))

				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					slideControl.setCustom(1450);
				})
				.waitSeconds(.25)
				.UNSTABLE_addTemporalMarkerOffset(0,()->{
					clawControl.toggleAutoOpenClose();
				})
				.waitSeconds(0.15)
				.UNSTABLE_addTemporalMarkerOffset(0.25,()->{

					clawControl.toggleWristRotate();
					slideControl.setIntakeOrGround();
					armControl.setIntake();
				})
				*/
				.waitSeconds(0.25)
				.addTemporalMarker(() -> {
					if(tagUse == 1){
						Trajectory zoneOne = bot.trajectoryBuilder(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(angle)))
								.lineToLinearHeading(new Pose2d(46,22,91))
								.build();
						bot.followTrajectoryAsync(zoneOne);
					}else if(tagUse == 3){
					Trajectory zoneThree = bot.trajectoryBuilder(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(angle)))
							.lineToLinearHeading(new Pose2d(46,-25,90))
							.build();
						bot.followTrajectoryAsync(zoneThree);
					}
				})
				.build();





		scan();

		waitForStart();


		bot.followTrajectorySequenceAsync(junction);
		//bot.followTrajectorySequenceAsync(goToConeStack);
		//bot.followTrajectorySequence(conesToJunction);



		while(opModeIsActive()  && !isStopRequested()){
			bot.update();
			armControl.update(telemetry);
			telemetry.addLine("run");
			telemetry.update();
			slideControl.update(telemetry);
		}

	}
}
