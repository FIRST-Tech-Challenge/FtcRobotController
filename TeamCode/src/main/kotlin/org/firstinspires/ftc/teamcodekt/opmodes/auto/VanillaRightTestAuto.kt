package org.firstinspires.ftc.teamcodekt.opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import ftc.rogue.blacksmith.Scheduler
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcodekt.components.AutoBotComponents
import org.firstinspires.ftc.teamcodekt.components.LiftConfig
import org.firstinspires.ftc.teamcodekt.components.createAutoBotComponents

class VanillaRightTestAuto : LinearOpMode() {
    lateinit var bot: AutoBotComponents

    private lateinit var mainTraj: TrajectorySequence

    override fun runOpMode() {
        bot = createAutoBotComponents()

        schedulePaths()

        waitForStart()

        Scheduler.launch(this) {
            bot.updateComponents()
            bot.drive.update()
            telemetry.update()
        }
    }

    private fun schedulePaths() = with(bot) {
        val liftOffsets = intArrayOf(
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_1,
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_2,
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_3,
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_4,
            AutoData.AUTO_INTAKE_LIFT_HEIGHT_5,
        )

        val startPose = Pose2d(`in`(91.0), `in`(-159.0), rad(90.0))
        drive.poseEstimate = startPose

        val builder = drive.trajectorySequenceBuilder(startPose)

        builder
            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                lift.goToHigh()
                wrist.setToForwardsPos()
                arm.setToForwardsPos()
            }

            .splineTo(Vector2d(`in`(91.0), `in`(-50.0)), rad(90.0))
            .setTurnConstraint(Math.toRadians(430.0), Math.toRadians(125.0))
            .splineTo(
                Vector2d(`in`(AutoData.DEPOSIT_X + .68), `in`(AutoData.DEPOSIT_Y + 0.57)),
                rad(AutoData.DEPOSIT_ANGLE - 2.25)
            )
            .resetTurnConstraint()

            .UNSTABLE_addTemporalMarkerOffset(AutoData.LOWER_OFFSET) {
                lift.height -= AutoData.DEPOSIT_DROP_AMOUNT
            }

            .UNSTABLE_addTemporalMarkerOffset(AutoData.DEPOSIT_OFFSET) {
                claw.openForDeposit() // Deposit the cone while turning
            }

            .waitSeconds(AutoData.DEPOSIT_DELAY + .05)

        for (i in 0..3) {
            builder
                .UNSTABLE_addTemporalMarkerOffset(AutoData.RETRACT_OFFSET) {
                    claw.openForIntakeWide()
                    lift.height = liftOffsets[i]

                    arm.setToBackwardsPosButLikeSliiiightlyHigher()
                    wrist.setToBackwardsPos()
                }

                .setReversed(true)
                .splineTo(
                    Vector2d(
                        `in`(AutoData.INTAKE_X + 1.073),
                        `in`(AutoData.INTAKE_Y + 1.897)
                    ), rad(0.0)
                )

                .UNSTABLE_addTemporalMarkerOffset(-0.02) {
                    claw.close()
                }

                .UNSTABLE_addTemporalMarkerOffset(.05) {
                    lift.height = LiftConfig.HIGH
                }

                .UNSTABLE_addTemporalMarkerOffset(.07) {
                    arm.setToForwardsPos()
                    wrist.setToForwardsPos()
                }

                .waitSeconds(.37 + i * .07)

                .setReversed(false)

            if (i == 0) {
                builder.splineTo(
                    Vector2d(
                        `in`(AutoData.DEPOSIT_X + .512 + i * .045),
                        `in`(AutoData.DEPOSIT_Y + .738 - i * .045)
                    ),
                    rad(AutoData.DEPOSIT_ANGLE)
                )
            } else {
                builder.splineTo(
                    Vector2d(
                        `in`(AutoData.DEPOSIT_X + .556 + i * .045),
                        `in`(AutoData.DEPOSIT_Y + .6980 - i * .045)
                    ), rad(AutoData.DEPOSIT_ANGLE)
                )
            }

            builder
                .UNSTABLE_addTemporalMarkerOffset(AutoData.LOWER_OFFSET) {
                    lift.height -= AutoData.DEPOSIT_DROP_AMOUNT
                }

                .UNSTABLE_addTemporalMarkerOffset(AutoData.DEPOSIT_OFFSET) {
                    claw.openForDeposit() // Deposit the cone while turning
                }
                .waitSeconds(AutoData.DEPOSIT_DELAY)
        }
        builder
            .UNSTABLE_addTemporalMarkerOffset(AutoData.RETRACT_OFFSET) {
                intake.enable()
                claw.openForIntakeNarrow()

                lift.height = liftOffsets[4]

                arm.setToBackwardsPos()
                wrist.setToBackwardsPos()
            }

            .setReversed(true)
            .splineTo(Vector2d(`in`(AutoData.INTAKE_X + 0.15), `in`(AutoData.INTAKE_Y)), rad(0.0))

            .UNSTABLE_addTemporalMarkerOffset(-0.12) {
                intake.disable()
            }

            .UNSTABLE_addTemporalMarkerOffset(0.0) {
                claw.close()
            }

            .waitSeconds(.04)
    }

    private fun rad(degrees: Double): Double {
        return Math.toRadians(degrees)
    }

    private fun `in`(centimeters: Double): Double {
        return centimeters * 0.3837008
    }
}
