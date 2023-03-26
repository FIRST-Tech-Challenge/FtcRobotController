package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.MechanismTemplates.OdoPod;
import org.firstinspires.ftc.teamcode.MechanismTemplates.PoleAlignment;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetectionDeposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "STATES_splineAutoRightSideMamaTime")
public class SPLINEAutoRightSideKrishJayAllNighterGangGangGang extends PowerPlay_AprilTagDetectionDeposit {
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
    public static double openingX = 36;
    public static double openingY = 3;

    // [MEDIUM JUNCTION]
    public static double mediumX1 = 37.5; // 43.5
    public static double mediumY1 = 6.5;

    public static double mediumX2 = 37;
    public static double mediumY2 = 6.25;

    public static double mediumX3 = 35.25;
    public static double mediumY3 = 5.75;

    public static double mediumX4 = 33.75;
    public static double mediumY4 = 5.25; // -4.8

    public static double mediumX5 = 32;
    public static double mediumY5 = 5.25;

    // [PARKING]
    public static double zone2X = 47;
    public static double zone2Y = 0;
    public static double headingZone2 = 270;
    public static double zone1backwards = 25;
    public static double zone3forwards = 26; //24.5

    // [CONE STACK]
    public static double coneStackForward = 25.65;
    public static double coneStackForward2 = 25.85;
    public static double coneStackForward3 = 26.5;
    public static double coneStackForward4 = 26.2;
    public static double coneStackForward5 = 26.15;
    public static double xConeStack1 = 48; // 49.8
    public static double xConeStack2 = 46; // 47.5
    public static double xConeStack3 = 44.25;
    public static double xConeStack4 = 43.5; //45
    public static double xConeStack5 = 42.5;
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
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    slideControl.setMidJunction();
                    armControl.setCustom(armPosition);
                    clawControl.toggleWristRotate();
                })
                .lineToLinearHeading(new Pose2d(openingX, openingY, Math.toRadians(openingHeading)))
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    slideControl.setIntakeOrGround();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    armControl.setIntake();
                    clawControl.toggleWristRotate();
                })
                .waitSeconds(0.1)
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
                .waitSeconds(0.06) // claw
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
                    clawControl.toggleAutoOpenClose();
                })
                //.waitSeconds(0.03) // 0.1 old val,
                .waitSeconds(0.02) // 0.15 old val

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    armControl.setIntake();
                    clawControl.toggleWristRotate();
                })
                //.waitSeconds(0.25)


                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setCustom(530);//475
                })
                .splineTo(new Vector2d(xConeStack2, -coneStackForward2), Math.toRadians(coneStackEndTangent))
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
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.02)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    armControl.setIntake();
                    clawControl.toggleWristRotate();
                })


                //.waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setCustom(440);//340
                })
                .splineTo(new Vector2d(xConeStack3, -coneStackForward3), Math.toRadians(coneStackEndTangent))
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
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.02)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    armControl.setIntake();
                    clawControl.toggleWristRotate();
                })


                .waitSeconds(0.25)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setCustom(150);
                })
                .splineTo(new Vector2d(xConeStack4, -coneStackForward4), Math.toRadians(coneStackEndTangent))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.07)
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
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.02)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    armControl.setIntake();
                })
                //.waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    alignmentControl.toggleAlignmentDevice();
                    slideControl.setIntakeOrGround();
                })

                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    slideControl.setIntakeOrGround();//225
                })
                .splineTo(new Vector2d(xConeStack5, -coneStackForward5), Math.toRadians(coneStackEndTangent))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawControl.toggleAutoOpenClose();
                })
                .waitSeconds(0.06)
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
                    clawControl.toggleAutoOpenClose();
                })


                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {

                    armControl.setIntake();
                    clawControl.toggleWristRotate();
                    alignmentControl.lowerServo();
                })
                //.waitSeconds(0.25)


                .lineToLinearHeading(new Pose2d(zone2X,zone2Y,Math.toRadians(headingZone2)))

                .addTemporalMarker(() -> {

                    for (DcMotorEx motor : bot.motors)
                    {
                        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }

					if(tagUse == 1){
						TrajectorySequence zoneOne = bot.trajectorySequenceBuilder(new Pose2d(zone2X,zone2Y,Math.toRadians(headingZone2)))
                                .lineToLinearHeading(new Pose2d(zone2X ,zone2Y, Math.toRadians(headingZone2)))
                                .back(zone1backwards)
								.build();
						bot.followTrajectorySequenceAsync(zoneOne);
					}else if(tagUse == 3)
                    {
                        TrajectorySequence zoneTwo = bot.trajectorySequenceBuilder(new Pose2d(zone2X,zone2Y,Math.toRadians(headingZone2)))
                                .forward(zone3forwards)
                                .build();
                        bot.followTrajectorySequenceAsync(zoneTwo);
                    }

                /*
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
                */
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