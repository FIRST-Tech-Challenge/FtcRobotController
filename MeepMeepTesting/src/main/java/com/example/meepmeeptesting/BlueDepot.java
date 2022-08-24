package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//Red warehouse visualization
public class BlueDepot {
	public static void main(String[] args) {

		MeepMeep mm = new MeepMeep(800); //Creates a new MeepMeep object with window dimensions of 800x800 px
		RoadRunnerBotEntity bot = new DefaultBotBuilder(mm) //Creates a new Roadrunner bot
				.setColorScheme(new ColorSchemeBlueDark()) //Sets the bot's color to be dark blue
				.setConstraints(57.635630404559784, 38.7, 4.5836622, Math.toRadians(60), 14.2) //Sets the speed and acceleration constraints of the robot
				.setDimensions(13.2,16.603) //Sets the dimensions of the robot
				//All of the TrajectorySequenceBuilder functions that follow describe the path that the robot will follow
				.followTrajectorySequence(drive ->
					drive.trajectorySequenceBuilder(new Pose2d(-34, 63, Math.toRadians(90)))
						.addSpatialMarker(new Vector2d(-33,40),() -> { /* mechanisms.rotateArm(UtilityConstants.THIRD_LEVEL_POS,0.65) */ })
						.lineToSplineHeading(new Pose2d(-32.5,24,Math.toRadians(0)))
						.addDisplacementMarker(() -> { /* mechanisms.releaseServoMove(0.3); */ }).waitSeconds(0.5)
						.addDisplacementMarker(55,() -> { /* mechanisms.reset(); */ })
						.lineToSplineHeading(new Pose2d(-62,55,Math.toRadians(180)))
						.addDisplacementMarker(() -> { /* mechanisms.rotateCarousel(0.6); */ }).setReversed(true)
						.waitSeconds(1.0).splineToConstantHeading(new Vector2d(10,64),Math.toRadians(0))
						.splineToConstantHeading(new Vector2d(50,64),Math.toRadians(0))
						.build());

		mm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL) //Sets the MeepMeep object's background to be the official Freight Frenzy field
				.setDarkMode(true) //Self explanatory
				.setBackgroundAlpha(0.95f) //Sets the background opacity to 95%
				.addEntity(bot) //Adds the Roadrunner bot
				.start(); //Starts the MeepMeep
	}
}