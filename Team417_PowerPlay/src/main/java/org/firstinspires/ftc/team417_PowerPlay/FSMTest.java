package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="FSM Test")
public class FSMTest extends BaseAutonomous {
    enum State {
        PUSH_SIGNAL_CONE,
        SIDE_MID_JUNCTION,
        CLEAR_MID_JUNCTION,
        MOVE_TO_CONE_STACK,
        //TRAJECT_4,
        IDLE
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(-36, 64, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        initializeAuto();

        drive.setPoseEstimate(startPose);

        Trajectory pushSignalCone = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-36, 11), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-36, 36), Math.toRadians(90))
                .build();
        Trajectory sideMidJunction = drive.trajectoryBuilder(pushSignalCone.end())
                .splineToConstantHeading(new Vector2d(-23, 36), Math.toRadians(0)) // the end of scoring pre-loaded, them arm lower
                .build();

        Trajectory clearMidJunction = drive.trajectoryBuilder(sideMidJunction.end())
                .splineToConstantHeading(new Vector2d(-23, 40), Math.toRadians(180))// moving back a little
                .build();

        Trajectory moveToConeStack = drive.trajectoryBuilder(clearMidJunction.end(),false)// goes to cone stack
                .splineToConstantHeading(new Vector2d(-36,32), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-36, 22), Math.toRadians(-90))
                .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
                .build();

        /*Trajectory traject4 = drive.trajectoryBuilder(moveToConeStack.end(), false)
                .splineTo(new Vector2d(-36, 20), Math.toRadians(45)) //after this should score the first cone from stack
                .build();
        Trajectory traject5 = drive.trajectoryBuilder(traject4.end(), false)
                .splineTo(new Vector2d(-36, 20), Math.toRadians(45)) //goes back to collect second cone from stack.
                        .build();
*/
        waitForStart();

        if (isStopRequested()) return;

        grabberServo.setPosition(GRABBER_CLOSED);

        currentState = State.PUSH_SIGNAL_CONE;
        drive.followTrajectoryAsync(pushSignalCone);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case PUSH_SIGNAL_CONE:
                    if (Math.abs(motorArm.getCurrentPosition() - GROUND_JUNCT_ARM_POSITION) > ARM_ENCODER_TOLERANCE && opModeIsActive()) {
                        motorArm.setPower((GROUND_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER);
                    }
                    if (!drive.isBusy()) {
                        currentState = State.SIDE_MID_JUNCTION;
                        drive.followTrajectoryAsync(sideMidJunction);
                    }
                    break;
                case SIDE_MID_JUNCTION:
                    // MOVE ARM TO GROUND JUNCTION POSITION
                    if (Math.abs(motorArm.getCurrentPosition() - MID_JUNCT_ARM_POSITION) > ARM_ENCODER_TOLERANCE && opModeIsActive()) {
                        motorArm.setPower((MID_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER);
                    }
                    if (!drive.isBusy()) {
                        grabberServo.setPosition(GRABBER_OPEN);
                        currentState = State.CLEAR_MID_JUNCTION;
                        motorArm.setPower(0);
                        drive.followTrajectoryAsync(clearMidJunction);
                    }
                    break;
                case CLEAR_MID_JUNCTION:
                    if (Math.abs(motorArm.getCurrentPosition() - FIRST_CONE_STACK_ARM_POSITION) > ARM_ENCODER_TOLERANCE && opModeIsActive()) {
                        motorArm.setPower(-0.4);
                    } else {
                        motorArm.setPower((FIRST_CONE_STACK_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER) ;
                    }
                    if (!drive.isBusy()) {
                        currentState = State.MOVE_TO_CONE_STACK;
                        drive.followTrajectoryAsync(moveToConeStack);
                    }
                case MOVE_TO_CONE_STACK:
                    if (Math.abs(motorArm.getCurrentPosition() - FIRST_CONE_STACK_ARM_POSITION) > ARM_ENCODER_TOLERANCE && opModeIsActive()) {
                        motorArm.setPower(-0.4);
                    } else {
                        motorArm.setPower((FIRST_CONE_STACK_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER) ;
                    }
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                        motorArm.setPower(0);
                        //drive.followTrajectoryAsync(traject4);
                    }
                    break;
                /*case TRAJECT_4:
                    if (Math.abs(motorArm.getCurrentPosition() - LOW_JUNCT_ARM_POSITION) > ARM_ENCODER_TOLERANCE && opModeIsActive()) {
                        motorArm.setPower((LOW_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER);
                    }
                    if(!drive.isBusy()){
                        currentState = State.IDLE;
                        motorArm.setPower(0);
                    }*/
                case IDLE:
                    break;
            }
            drive.update();
        }
    }
}
