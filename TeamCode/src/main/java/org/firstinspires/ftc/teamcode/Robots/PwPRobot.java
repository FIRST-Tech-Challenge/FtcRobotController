package org.firstinspires.ftc.teamcode.Robots;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Aligner;
import org.firstinspires.ftc.teamcode.Components.CV.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.ClawExtension;
import org.firstinspires.ftc.teamcode.Components.Field;
import org.firstinspires.ftc.teamcode.Components.Lift;
import org.firstinspires.ftc.teamcode.Components.LiftArm;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class PwPRobot extends BasicRobot {
    private Aligner aligner = null;
    private Claw claw = null;
    private LiftArm liftArm = null;
    private ClawExtension clawExtension = null;
    private Lift lift = null;
    public Field field = null;
    public CVMaster cv = null;
    public SampleMecanumDrive roadrun = null;


    public PwPRobot(LinearOpMode opMode, boolean p_isTeleop) {
        super(opMode, p_isTeleop);
        roadrun = new SampleMecanumDrive(opMode.hardwareMap);
        cv = new CVMaster();
        field = new Field(roadrun, cv);
//        aligner = new Aligner();
        claw = new Claw();
        liftArm = new LiftArm();
//        clawExtension = new ClawExtension();
        lift = new Lift();

    }

    public void stop() {
        cv.stopCamera();
        logger.closeLog();
        op.stop();
    }

    public void autoAim() {
        if (queuer.queue(false, !roadrun.isBusy())) {
            if (!roadrun.isBusy() && field.lookingAtPole()) {
                /*double[] coords = cv.rotatedPolarCoordDelta();
                Trajectory trajectory = roadrun.trajectoryBuilder(roadrun.getPoseEstimate()).lineToLinearHeading(
                        new Pose2d(roadrun.getPoseEstimate().getX() + (coords[1] - 9) * cos(roadrun.getPoseEstimate().getHeading() + coords[0]),
                                roadrun.getPoseEstimate().getY() + (coords[1] - 9) * sin(roadrun.getPoseEstimate().getHeading() + coords[0]),
                                roadrun.getPoseEstimate().getHeading() + coords[0])).build();
                roadrun.followTrajectoryAsync(trajectory);*/
            }
        }
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        if (queuer.queue(false, !roadrun.isBusy())) {
            if (!roadrun.isBusy()) {
                roadrun.followTrajectorySequenceAsync(trajectorySequence);
            }
        }
    }
    public void followTrajectoryAsync (Trajectory trajectory) {
        if (queuer.queue(false, !roadrun.isBusy())) {
            if (!roadrun.isBusy()) {
                roadrun.followTrajectory(trajectory);
            }
        }
    }

    public void setFirstLoop(boolean value) {
        queuer.setFirstLoop(value);
    }

    public void openClaw() {
        if (queuer.queue(true, op.getRuntime() > claw.clawServoLastSwitchTime +
                claw.CLAW_SERVO_SWITCH_TIME)) {

            claw.clawServoLastSwitchTime = op.getRuntime();
            claw.openClaw();
        }
    }

    public void openClaw(boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, op.getRuntime() > claw.clawServoLastSwitchTime +
                claw.CLAW_SERVO_SWITCH_TIME)) {

            claw.clawServoLastSwitchTime = op.getRuntime();
            claw.openClaw();
        }
    }

    public void closeClaw() {
        if (queuer.queue(true, op.getRuntime() > claw.clawServoLastSwitchTime +
                claw.CLAW_SERVO_SWITCH_TIME)) {

            claw.clawServoLastSwitchTime = op.getRuntime();
            claw.closeClaw();
        }
    }

    public void toggleClawPosition() {
        if (queuer.queue(true, op.getRuntime() > claw.clawServoLastSwitchTime +
                claw.CLAW_SERVO_SWITCH_TIME)) {

            claw.clawServoLastSwitchTime = op.getRuntime();
            claw.toggleClawPosition();
        }
    }

    public void toggleClawPosition(boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, op.getRuntime() > claw.clawServoLastSwitchTime +
                claw.CLAW_SERVO_SWITCH_TIME)) {
            claw.toggleClawPosition();
        }
    }

    public void closeClaw(boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, op.getRuntime() > claw.clawServoLastSwitchTime +
                claw.CLAW_SERVO_SWITCH_TIME)) {

            claw.clawServoLastSwitchTime = op.getRuntime();
            claw.closeClaw();
        }
    }

    public boolean isConeReady() {
        return claw.isConeReady();
    }


    public void liftToPosition(Lift.LiftConstants targetJunction) {
        if (queuer.queue(false, Math.abs(targetJunction.getValue() - lift.getLiftPosition()) < 10 && lift.getLiftVelocity() == 0)) {
            lift.liftToPosition(targetJunction);
        }
    }

    public void liftToPosition(int tickTarget) {
        if (queuer.queue(false, Math.abs(tickTarget - lift.getLiftPosition()) < 10 && lift.getLiftVelocity() == 0)) {
            lift.liftToPosition(tickTarget);
        }
    }

    public void setLiftPower(double p_power) {
        lift.setLiftPower(p_power);
    }

    public void lowerLiftArmToIntake() {
        if (queuer.queue(true, op.getRuntime() > liftArm.liftArmServoLastSwitchTime +
                liftArm.LIFT_ARM_SERVO_SWITCH_TIME)) {

            liftArm.liftArmServoLastSwitchTime = op.getRuntime();
            liftArm.lowerLiftArmToIntake();
        }
    }

    public void lowerLiftArmToIntake(boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, op.getRuntime() > liftArm.liftArmServoLastSwitchTime +
                liftArm.LIFT_ARM_SERVO_SWITCH_TIME)) {

            liftArm.liftArmServoLastSwitchTime = op.getRuntime();
            liftArm.lowerLiftArmToIntake();
        }
    }

    public void raiseLiftArmToOuttake() {
        if (queuer.queue(true, op.getRuntime() > liftArm.liftArmServoLastSwitchTime +
                liftArm.LIFT_ARM_SERVO_SWITCH_TIME)) {

            liftArm.liftArmServoLastSwitchTime = op.getRuntime();
            liftArm.raiseLiftArmToOuttake();
        }
    }

    public void raiseLiftArmToOuttake(boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, op.getRuntime() > liftArm.liftArmServoLastSwitchTime +
                liftArm.LIFT_ARM_SERVO_SWITCH_TIME)) {

            liftArm.liftArmServoLastSwitchTime = op.getRuntime();
            liftArm.raiseLiftArmToOuttake();
        }
    }

    public void toggleArmPosition() {
        if (queuer.queue(true, op.getRuntime() > liftArm.liftArmServoLastSwitchTime +
                liftArm.LIFT_ARM_SERVO_SWITCH_TIME)) {

            liftArm.liftArmServoLastSwitchTime = op.getRuntime();
            liftArm.toggleArmPosition();
        }
    }

//    public void spinAlignerIntake() {
//        if (queuer.queue(true, alignerSensor.getSensorDistance() <
//                aligner.CONE_IN_ALIGNER_DISTANCE)) {
//
//            aligner.spinAlignerIntake();
//        }
//    }
//
//    public void stopAlignerIntake() {
//        if (queuer.queue(true, op.getRuntime() > aligner.alignerMotorLastStopTime +
//                aligner.ALIGNER_MOTOR_STOP_TIME)) {
//
//            aligner.stopAlignerIntake();
//        }
//    }
//
//    public void reverseAlignerIntake() {
//        if (queuer.queue(true, alignerSensor.getSensorDistance() >
//                aligner.CONE_OUT_OF_ALIGNER_DISTANCE)) {
//
//            aligner.reverseAlignerIntake();
//        }
//    }
//    public void reverseAlignerIntake() {
//        if (queuer.queue(true, !aligner.isConeInAligner())){
//
//            aligner.reverseAlignerIntake();
//        }
//    }

    public void teleOp() {
        //omnidirectional movement + turning


        //manual lift up/down
        if (op.gamepad2.right_trigger != 0 || op.gamepad2.left_trigger != 0) {
            lift.setLiftPower(op.gamepad2.right_trigger - op.gamepad2.left_trigger);
        }
        //when not manual lifting, automate lifting
        else {
            lift.setLiftPower(0);
//            lift.liftToTarget();
        }
//        op.telemetry.addData("centerOffset", cv.centerOfPole());
//        op.telemetry.addData("centerSize", cv.poleSize());
//        op.telemetry.addData("cvtheta",cv.rotatedPolarCoordDelta()[0]);
//        op.telemetry.addData("cvdistance",cv.rotatedPolarCoordDelta()[1]);
//        op.telemetry.addData("x", roadrun.getPoseEstimate().getX());
//        op.telemetry.addData("y", roadrun.getPoseEstimate().getY());
//        op.telemetry.addData("heading", roadrun.getPoseEstimate().getHeading()*180/PI);
//        if (op.gamepad1.x && !roadrun.isBusy() && field.lookingAtPole()) {
//            setFirstLoop(true);
//            queuer.reset();
//            autoAim();
//            setFirstLoop(false);
//        } else if (roadrun.isBusy()) {
//            autoAim();
//        } else {
//            queuer.reset();
            roadrun.setWeightedDrivePower(
                    new Pose2d(
                            -op.gamepad1.left_stick_y,
                            op.gamepad1.left_stick_x,
                            op.gamepad1.right_stick_x
                    )
            );
//        }
        //toggle automate lift target to higher junc
        if (op.gamepad2.dpad_up) {
            lift.toggleLiftPosition(1);
        }
        //toggle automate lift target to lower junc
        if (op.gamepad2.dpad_down) {
            lift.toggleLiftPosition(-1);
        }
        //toggle liftArm position
        if (op.gamepad2.x) {
            liftArm.raiseLiftArmToOuttake();
        }
        if (op.gamepad2.y) {
            liftArm.lowerLiftArmToIntake();
        }
        //manual open/close claw (will jsut be open claw in the future)
        if (op.gamepad1.x) {
            claw.openClaw();
        }
        roadrun.update();
        //will only close when detect cone
        //claw.closeClaw
        if (op.gamepad1.y) {
            claw.closeClaw();
        }
        claw.logClawStates();
    }
}
