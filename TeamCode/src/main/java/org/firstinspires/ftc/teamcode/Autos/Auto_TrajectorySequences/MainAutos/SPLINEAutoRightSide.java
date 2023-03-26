package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.MechanismTemplates.OdoPod;
import org.firstinspires.ftc.teamcode.MechanismTemplates.PoleAlignment;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetectionDeposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "STATES_splineAutoRightSide")
public class SPLINEAutoRightSide extends PowerPlay_AprilTagDetectionDeposit {
    public static double endTangent1 = 40;
    public static double openingStrafe = 9.25;
    // [MECHANISMS]
    private Arm armControl;
    public static int armPosition = 278;
    private Slide slideControl;
    private Claw clawControl;

    // [POLE ALIGNMENT]
    private PoleAlignment alignmentControl;

    // [OPENING MOVE]
    public static double openingX = 36.5;
    public static double openingY = 3.5;

    // [MEDIUM JUNCTION]
    public static double mediumX1 = 37.5; // 43.5
    public static double mediumY1 = 6.5;

    public static double mediumX2 = 37.5;
    public static double mediumY2 = 6.5;

    public static double mediumX3 = 34.75;
    public static double mediumY3 = 5.75;

    public static double mediumX4 = 34.75;
    public static double mediumY4 = 5.75; // -4.8

    public static double mediumX5 = 33;
    public static double mediumY5 = 5;

    // [PARKING]
    public static double zone2X = 46;
    public static double zone2Y = 2;
    public static double headingZone2 = 270;
    public static double zone1backwards = 25;
    public static double zone3forwards = 26; //24.5

    // [CONE STACK]
    public static double coneStackForward = 25.65;
    public static double xConeStack1 = 48; // 49.8
    public static double xConeStack2 = 46; // 47.5
    public static double xConeStack3 = 45;
    public static double xConeStack4 = 45;
    public static double xConeStack5 = 47.3;
    public static double coneStackHeading = 89;
    public static double coneStackEndTangent = -80;

    // [OPENING MOVE --> MEDIUM JUNCTION]
    public static double openingHeading = 270;
    // [1] MEDIUM JUNCTION --> CONE STACK
    public static double coneStackHeading1 = 93;
    // [1] CONE STACK --> MEDIUM JUNCTION
    public static double mediumHeading1 = 140;

    // [2] MEDIUM JUNCTION --> CONE STACK
    public static double coneStackHeading2 = 45;
    // [2] CONE STACK --> MEDIUM JUNCTION
    public static double mediumHeading2 = 131;

    // [3] MEDIUM JUNCTION --> CONE STACK
    public static double coneStackHeading3 = 45;
    // [3] CONE STACK --> MEDIUM JUNCTION
    public static double mediumHeading3 = 128;

    // [4] MEDIUM JUNCTION --> CONE STACK
    public static double coneStackHeading4 = 45;
    // [4] CONE STACK --> MEDIUM JUNCTION
    public static double mediumHeading4 = 131; // 230 old val for all


    public void initialize() {
        armControl = new Arm(hardwareMap);
        slideControl = new Slide(hardwareMap);
        alignmentControl = new PoleAlignment(hardwareMap);
    }

    public void scan() {
        super.runOpMode();
    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setPoseEstimate(startPose);
        initialize();

        TrajectorySequence junction = bot.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl = new Claw(hardwareMap);
                    OdoPod odoControl = new OdoPod(hardwareMap);
                })
                .waitSeconds(0.35)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    slideControl.setMidJunction();
                    armControl.setCustom(armPosition);
                    clawControl.toggleWristRotate();
                })
                .lineToLinearHeading(new Pose2d(openingX, openingY, Math.toRadians(openingHeading)))
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    slideControl.setIntakeOrGround();
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    armControl.setIntake();
                    clawControl.toggleWristRotate();
                })
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setCustom(735);//635
                })

                // GOING BACK TO CONE STACK
                .lineToLinearHeading(new Pose2d(xConeStack1, -1, Math.toRadians(270)))
                .forward(coneStackForward)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    alignmentControl.raiseServo();
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.12) // claw
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setCustom(1370);
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    slideControl.setCustom(950); //1050
                    armControl.setAutoExtake();
                    clawControl.toggleWristRotate();
                })
                .setReversed(true)
                .splineTo(new Vector2d(mediumX1, mediumY1), Math.toRadians(mediumHeading1))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setIntakeOrGround();
                })
                .waitSeconds(0.03) // 0.1 old val,
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.1) // 0.15 old val

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    armControl.setIntake();
                    clawControl.toggleWristRotate();
                })
                //.waitSeconds(0.25)


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setCustom(530);//475
                })
                .splineTo(new Vector2d(xConeStack2, -coneStackForward), Math.toRadians(coneStackEndTangent))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.03)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setCustom(1275);
                })
                .waitSeconds(0.1)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    slideControl.setCustom(950); //1050 for medium
                    armControl.setAutoExtake();
                    clawControl.toggleWristRotate();
                })
                .setReversed(true)
                .splineTo(new Vector2d(mediumX2, mediumY2), Math.toRadians(mediumHeading2))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setIntakeOrGround();
                })
                .waitSeconds(0.03)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.1)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    armControl.setIntake();
                    clawControl.toggleWristRotate();
                })


                //.waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setCustom(440);//340
                })
                .splineTo(new Vector2d(xConeStack3, -coneStackForward), Math.toRadians(coneStackEndTangent))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.03)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setCustom(1275);//1275
                })
                .waitSeconds(0.1)


                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    slideControl.setCustom(950);//1050
                    armControl.setAutoExtake();
                    clawControl.toggleWristRotate();
                })
                .setReversed(true)
                .splineTo(new Vector2d(mediumX3, mediumY3), Math.toRadians(mediumHeading3))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setIntakeOrGround();
                })
                .waitSeconds(0.1)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.02)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    armControl.setIntake();
                    clawControl.toggleWristRotate();
                })


                .waitSeconds(0.25)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setCustom(225);
                })
                .splineTo(new Vector2d(xConeStack4, -coneStackForward), Math.toRadians(coneStackEndTangent))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.03)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setCustom(1275);//1275
                })
                .waitSeconds(0.1)

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    slideControl.setCustom(880);//1050
                    armControl.setAutoExtake();
                    clawControl.toggleWristRotate();
                })
                .setReversed(true)
                .splineTo(new Vector2d(mediumX4, mediumY4), Math.toRadians(mediumHeading4))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setIntakeOrGround();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.02)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    alignmentControl.lowerServo();
                    armControl.setIntake();
                })
                /*
                //.waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    alignmentControl.toggleAlignmentDevice();
                    slideControl.setIntakeOrGround();
                })

                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setIntakeOrGround();//225
                })
                .splineTo(new Vector2d(xConeStack5, -coneStackForward), Math.toRadians(coneStackEndTangent))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.03)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setCustom(1275);//1275
                })
                .waitSeconds(0.1)


                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    slideControl.setCustom(880);// 1050
                    armControl.setAutoExtake();
                    clawControl.toggleWristRotate();
                })
                .setReversed(true)
                .splineTo(new Vector2d(mediumX5, mediumY5), Math.toRadians(mediumHeading4))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setIntakeOrGround();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.02)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    armControl.setIntake();
                    clawControl.toggleWristRotate();
                })
                //.waitSeconds(0.25)
                 */

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    alignmentControl.lowerServo();
                })
                .waitSeconds(1)
                .back(-3)
                .lineToLinearHeading(new Pose2d(zone2X,zone2Y,Math.toRadians(headingZone2)))
                .addTemporalMarker(() -> {
				/*
					if(tagUse == 1){
						TrajectorySequence zoneOne = bot.trajectorySequenceBuilder(new Pose2d(mediumX4,mediumY4,Math.toRadians(mediumHeading4)))
								.lineToLinearHeading(new Pose2d(51 ,27, Math.toRadians(89)))
								.build();
						bot.followTrajectorySequenceAsync(zoneOne);
					}else if(tagUse == 2) {
						TrajectorySequence zoneTwo = bot.trajectorySequenceBuilder(new Pose2d(mediumX4,mediumY4,Math.toRadians(mediumHeading4)))
								.lineToLinearHeading(new Pose2d(51 ,0, Math.toRadians(89)))
								.build();
						bot.followTrajectorySequenceAsync(zoneTwo);
					}else{
						Trajectory zoneThree = bot.trajectoryBuilder(new Pose2d(mediumX4,mediumY4,Math.toRadians(mediumHeading4)))
								.lineToLinearHeading(new Pose2d(51 ,-23, Math.toRadians(89)))
								.build();
						bot.followTrajectoryAsync(zoneThree);
					}
					*/

                  if (tagUse == 1) {
                        Trajectory backward1 = bot.trajectoryBuilder(new Pose2d(zone2X, zone2Y, Math.toRadians(headingZone2)))
                                .back(zone1backwards)
                                .build();
                        bot.followTrajectoryAsync(backward1);
                    } else if(tagUse == 3){
                        Trajectory forward3 = bot.trajectoryBuilder(new Pose2d(zone2X, zone2Y, Math.toRadians(headingZone2)))
                                .forward(zone3forwards)
                                .build();

                        bot.followTrajectoryAsync(forward3);
                    }

                })

                .build();
        scan();
        waitForStart();
        bot.followTrajectorySequenceAsync(junction);
        while (opModeIsActive() && !isStopRequested()) {
            bot.update();
            armControl.update(telemetry);
            slideControl.update(telemetry);
        }
    }
}
// EAT CHICKEN ┻━┻ ︵ヽ(`Д´)ﾉ︵ ┻━┻