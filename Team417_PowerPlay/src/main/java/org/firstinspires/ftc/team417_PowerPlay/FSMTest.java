package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="FSM Test")
public class FSMTest extends BaseAutonomous {
    enum State {
        TRAJECT_1,
        TRAJECT_2,
        TRAJECT_3,
        TRAJECT_4,
        IDLE
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(-36, 64, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        initializeAuto();

        drive.setPoseEstimate(startPose);

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-36, 10), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-36, 45), Math.toRadians(-90))
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .splineToConstantHeading(new Vector2d(-24, 36), Math.toRadians(0))
                .build();

        Trajectory traject3 = drive.trajectoryBuilder(traject2.end(),false)
                .splineToConstantHeading(new Vector2d(-36,20), Math.toRadians(-90))
                .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
                .build();

        Trajectory traject4 = drive.trajectoryBuilder(traject3.end(), false)
                .splineTo(new Vector2d(-36, 20), Math.toRadians(45))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.TRAJECT_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case TRAJECT_1:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECT_2;
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case TRAJECT_2:
                    // MOVE ARM TO GROUND JUNCTION POSITION
                    if (Math.abs(motorArm.getCurrentPosition() - GROUND_JUNCT_ARM_POSITION) > ARM_ENCODER_TOLERANCE && opModeIsActive()) {
                        motorArm.setPower((GROUND_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER);
                    }
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECT_3;
                        motorArm.setPower(0);
                        drive.followTrajectoryAsync(trajectory3);
                    }
                    break;
                case TRAJECT_3:
                    if (Math.abs(motorArm.getCurrentPosition() - MID_JUNCT_ARM_POSITION) > ARM_ENCODER_TOLERANCE && opModeIsActive()) {
                        motorArm.setPower((MID_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER);
                    }
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECT_4;
                        motorArm.setPower(0);
                        drive.followTrajectoryAsync(trajectory4);
                    }
                    break;
                case TRAJECT_4:
                    if (Math.abs(motorArm.getCurrentPosition() - LOW_JUNCT_ARM_POSITION) > ARM_ENCODER_TOLERANCE && opModeIsActive()) {
                        motorArm.setPower((LOW_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER);
                    }
                    if(!drive.isBusy()){
                        currentState = State.IDLE;
                        motorArm.setPower(0);
                    }
                case IDLE:
                    break;
            }
            drive.update();
        }
    }
}
