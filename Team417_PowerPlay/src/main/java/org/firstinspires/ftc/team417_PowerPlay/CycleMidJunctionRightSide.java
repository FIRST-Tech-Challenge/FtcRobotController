package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name="FSM new")
public class CycleMidJunctionRightSide extends BaseAutonomous {
    ElapsedTime time;
    enum State {
        GO_TO_MID_JUNCTION,
        SIDE_MID_JUNCTION,
        CLEAR_MID_JUNCTION,
        MOVE_TO_CONE_STACK,
        RAISE_ARM,
        CYCLE_1,
        IDLE
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(-36, 64, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        initializeAuto();
        time = new ElapsedTime();

        drive.setPoseEstimate(startPose);

        Trajectory goToMidJunction = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-36, 30), Math.toRadians(-90))
                .build();
        Trajectory sideMidJunction = drive.trajectoryBuilder(goToMidJunction.end())
                .splineToConstantHeading(new Vector2d(-36, 42), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-30, 30, Math.toRadians(-45)), Math.toRadians(-45))
                .build();

        Trajectory clearMidJunction = drive.trajectoryBuilder(sideMidJunction.end())
                .splineToSplineHeading(new Pose2d(-36, 38, Math.toRadians(-90)), Math.toRadians(90))// moving back a little
                .splineToSplineHeading(new Pose2d(-36, 20, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        Trajectory moveToConeStack = drive.trajectoryBuilder(clearMidJunction.end(),false)// goes to cone stack
                .splineToSplineHeading(new Pose2d(-60,12, Math.toRadians(180)), Math.toRadians(180))
                .build();

        Trajectory cycle1 = drive.trajectoryBuilder(moveToConeStack.end(), false)
                .splineToConstantHeading(new Vector2d(-40, 12), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-30, 14, Math.toRadians(45)), Math.toRadians(0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        grabberServo.setPosition(GRABBER_CLOSED);
        sleep(800);

        currentState = State.GO_TO_MID_JUNCTION;
        drive.followTrajectoryAsync(goToMidJunction);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case GO_TO_MID_JUNCTION:

                    if (!drive.isBusy()) {
                        currentState = State.SIDE_MID_JUNCTION;
                        drive.followTrajectoryAsync(sideMidJunction);
                    }
                    break;
                case SIDE_MID_JUNCTION:
                    if (drive.isBusy()) {
                        if (Math.abs(motorArm.getCurrentPosition() - MID_JUNCT_ARM_POSITION) > ARM_ENCODER_TOLERANCE && opModeIsActive()) {
                            motorArm.setPower((MID_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER);
                        }
                    } else {
                        while (motorArm.getCurrentPosition() > 1000) {
                            motorArm.setPower(-0.4);
                            telemetry.addLine("lowering arm");
                            telemetry.addData("arm current position", motorArm.getCurrentPosition());
                            telemetry.update();
                        }
                        telemetry.addLine("Done lowering arm");
                        telemetry.update();
                        motorArm.setPower(0);
                        grabberServo.setPosition(GRABBER_OPEN);
                        currentState = State.CLEAR_MID_JUNCTION;
                        drive.followTrajectoryAsync(clearMidJunction);

                        /*if (motorArm.getCurrentPosition() > 1000) {
                            motorArm.setPower(-0.4);
                            telemetry.addLine("lowering arm");
                            telemetry.addData("arm current position", motorArm.getCurrentPosition());
                            telemetry.update();
                        } else {
                            motorArm.setPower(0);
                            grabberServo.setPosition(GRABBER_OPEN);
                            currentState = State.CLEAR_MID_JUNCTION;
                            drive.followTrajectoryAsync(clearMidJunction);
                        }*/
                    }

                case CLEAR_MID_JUNCTION:
                    if (!drive.isBusy()) {
                        currentState = State.MOVE_TO_CONE_STACK;
                        drive.followTrajectoryAsync(moveToConeStack);
                    }
                case MOVE_TO_CONE_STACK:
                    if (Math.abs(motorArm.getCurrentPosition() - FIRST_CONE_STACK_ARM_POSITION) > ARM_ENCODER_TOLERANCE && opModeIsActive()) {
                        motorArm.setPower((FIRST_CONE_STACK_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER);
                    }
                    if (!drive.isBusy()) {
                        grabberServo.setPosition(GRABBER_CLOSED);
                        sleep(200);
                        motorArm.setPower(0);

                        currentState = State.CYCLE_1;
                        drive.followTrajectoryAsync(cycle1);
                    }
                case CYCLE_1:
                    if (Math.abs(motorArm.getCurrentPosition() - MID_JUNCT_ARM_POSITION) > ARM_ENCODER_TOLERANCE && opModeIsActive()) {
                        motorArm.setPower((MID_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER);
                    }
                    if (!drive.isBusy()) {
                        motorArm.setPower(0);
                        grabberServo.setPosition(GRABBER_OPEN);
                        currentState = State.IDLE;
                    }
                case IDLE:
                    break;
            }
            drive.update();
        }
    }
}
