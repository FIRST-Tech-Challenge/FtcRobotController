//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
//import org.firstinspires.ftc.teamcodekt.opmodes.auto.RogueBaseAuto;
//
//@SuppressWarnings("CodeBlock2Expr")
//@Autonomous
//public class LM2_Right_v1 extends RogueBaseAuto {
//    private Runnable armPosFunction;
//    private Runnable wristPosFunction;
//
//    // Variable added to signal when to start the parking sequence
//    private boolean startParking = false;
//
//    private TrajectorySequence mainTraj, parkTraj;
//
//    @Override
//    public void execute() {
//        schedulePaths();
//
//        // Added this during init
//        int signalZone = getBot().getCamera().waitForStartWithVision();
//        telemetry.addData("Final signal zone", signalZone);
//        telemetry.update();
//
//        TrajectorySequenceBuilder parkTrajBuilder = drive.trajectorySequenceBuilder(mainTraj.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                    lift.setHeight(RobotConstants.Lift.HIGH);
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
//                    armPosFunction = arm::setToForwardsAutoPos;
//                    wristPosFunction = wrist::setToForwardsPos;
//                })
//
//                .waitSeconds(0.375)
//
//                .setReversed(false)
//                .splineTo(new Vector2d(in(AutoData.DEPOSIT_X + .775 + .25), in(AutoData.DEPOSIT_Y + .475 - .25)), rad(AutoData.DEPOSIT_ANGLE + AutoData.DEPOSIT_ANGLE_ADJUSTMENT * 5 - .39))
//
//                .UNSTABLE_addTemporalMarkerOffset(AutoData.LOWER_OFFSET, () -> {
//                    lift.setHeight(RobotConstants.Lift.HIGH - AutoData.DEPOSIT_DROP_AMOUNT);
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(AutoData.DEPOSIT_OFFSET, () -> {
//                    claw.openForDeposit(); // Deposit the cone while turning
//                })
//
//                .waitSeconds(AutoData.DEPOSIT_DELAY)
//
//                .UNSTABLE_addTemporalMarkerOffset(AutoData.RETRACT_OFFSET, () -> {
//                    lift.goToZero();
//
//                    armPosFunction = arm::setToRestingPos;
//                    wristPosFunction = wrist::setToRestingPos;
//                })
//
//                .addTemporalMarker(() -> {
//                    claw.close();
//                });
//
//
//        switch (signalZone) {
//            case 1:
//                parkTrajBuilder
//                        .back(in(8))
//                        .turn(rad(45.25))
//                        .forward(in(67));
//                break;
//            case 2:
//                parkTrajBuilder
//                        .back(in(10))
//                        .turn(rad(47.5));
//                break;
//            default:
//                parkTrajBuilder
//                        .setReversed(true)
//                        .splineTo(new Vector2d(in(AutoData.INTAKE_X - 3), in(AutoData.INTAKE_Y + 2)), rad(0));
//                break;
//        }
//
//        parkTraj = parkTrajBuilder.build();
//
//        waitForStart();
//
//        Scheduler.start(this, () -> {
//            arm.update(telemetry, false);
//
//            lift.update(telemetry, RobotConstants.Lift.USE_AGGRESSIVE_ASCENDANCE);
//            wrist.update();
//            drive.update();
//
//            if (startParking) {
//                startParking = false;
//                drive.followTrajectorySequenceAsync(parkTraj);
//            }
//
//            telemetry.update();
//        });
//    }
//
//    public void schedulePaths() {
//        int[] liftOffsets = {
//                RobotConstants.Lift.AUTO_INTAKE_1,
//                RobotConstants.Lift.AUTO_INTAKE_2,
//                RobotConstants.Lift.AUTO_INTAKE_3,
//                RobotConstants.Lift.AUTO_INTAKE_4,
//        };
//
//        Pose2d startPose = new Pose2d(in(91), in(-159), rad(90));
//        drive.setPoseEstimate(startPose);
//
//        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
//
//        builder
//                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                    lift.setHeight(RobotConstants.Lift.HIGH);
//                    wristPosFunction = wrist::setToForwardsPos;
//                    armPosFunction = arm::setToForwardsAutoPos;
//                })
//
//                .splineTo(new Vector2d(in(91), in(-50)), rad(90))
//                .setTurnConstraint(Math.toRadians(430), Math.toRadians(125))
//                .splineTo(new Vector2d(in(AutoData.DEPOSIT_X + .68), in(AutoData.DEPOSIT_Y + 0.57)), rad(AutoData.DEPOSIT_ANGLE - 2.25))
//                .resetTurnConstraint()
//
//                .UNSTABLE_addTemporalMarkerOffset(AutoData.LOWER_OFFSET, () -> {
//                    lift.setHeight(RobotConstants.Lift.HIGH - AutoData.DEPOSIT_DROP_AMOUNT);
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(AutoData.DEPOSIT_OFFSET, () -> {
//                    claw.openForDeposit(); // Deposit the cone while turning
//                })
//
//                .waitSeconds(AutoData.DEPOSIT_DELAY + .05);
//
//        for (int i = 0; i < 4; i++) {
//            int finalI = i;
//
//            builder
//                    .UNSTABLE_addTemporalMarkerOffset(AutoData.RETRACT_OFFSET, () -> {
//                        claw.openForIntakeWide();
//                        lift.setHeight(liftOffsets[finalI]);
//
//                        armPosFunction = arm::setToBackwardsAutoPos;
//                        wristPosFunction = wrist::setToBackwardsPos;
//                    })
//
//                    .setReversed(true)
//                    .splineTo(new Vector2d(in(AutoData.INTAKE_X + 1.073), in(AutoData.INTAKE_Y + 1.897)), rad(0))
//
//                    .UNSTABLE_addTemporalMarkerOffset(-0.02, () -> {
//                        claw.close();
//                    })
//
//                    .UNSTABLE_addTemporalMarkerOffset(.05, () -> {
//                        lift.setHeight(RobotConstants.Lift.HIGH);
//                    })
//
//                    .UNSTABLE_addTemporalMarkerOffset(.07, () -> {
//                        armPosFunction = arm::setToForwardsAutoPos;
//                        wristPosFunction = wrist::setToForwardsPos;
//                    })
//
//                    .waitSeconds(.37 + i * .07)
//
//                    .setReversed(false);
//
//            if (i == 0) {
//                builder.splineTo(new Vector2d(in(AutoData.DEPOSIT_X + .512 + i * .045), in(AutoData.DEPOSIT_Y + .738 - i * .045)), rad(AutoData.DEPOSIT_ANGLE + AutoData.DEPOSIT_ANGLE_ADJUSTMENT * i - 2.01 - .89));
//            } else {
//                builder.splineTo(new Vector2d(in(AutoData.DEPOSIT_X + .556 + i * .045), in(AutoData.DEPOSIT_Y + .6980 - i * .045)), rad(AutoData.DEPOSIT_ANGLE + AutoData.DEPOSIT_ANGLE_ADJUSTMENT * i - 1.91));
//            }
//
//            builder
//                    .UNSTABLE_addTemporalMarkerOffset(AutoData.LOWER_OFFSET, () -> {
//                        lift.setHeight(RobotConstants.Lift.HIGH - AutoData.DEPOSIT_DROP_AMOUNT);
//                    })
//
//                    .UNSTABLE_addTemporalMarkerOffset(AutoData.DEPOSIT_OFFSET, () -> {
//                        claw.openForDeposit(); // Deposit the cone while turning
//                    })
//
//                    .waitSeconds(AutoData.DEPOSIT_DELAY);
//        }
//
//        builder
//                .UNSTABLE_addTemporalMarkerOffset(AutoData.RETRACT_OFFSET, () -> {
//                    intake.enable();
//                    claw.openForIntakeNarrow();
//
//                    lift.setHeight(RobotConstants.Lift.AUTO_INTAKE_5);
//
//                    armPosFunction = arm::setToBackwardsAutoPos;
//                    wristPosFunction = wrist::setToBackwardsPos;
//                })
//
//                .setReversed(true)
//                .splineTo(new Vector2d(in(AutoData.INTAKE_X + 0.15), in(AutoData.INTAKE_Y)), rad(0))
//
//                .UNSTABLE_addTemporalMarkerOffset(-0.12, () -> {
//                    intake.disable();
//                })
//
//                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                    claw.close();
//                })
//
//                .waitSeconds(.04)
//
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    startParking = true;
//                });
//
//        drive.followTrajectorySequenceAsync(mainTraj = builder.build());
//    }
//
//    public static double rad(double degrees) {
//        return Math.toRadians(degrees);
//    }
//
//    public static double in(double centimeters) {
//        return centimeters * 0.3837008;
//    }
//}
