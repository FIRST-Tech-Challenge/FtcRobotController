package org.firstinspires.ftc.teamcode.Robots;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_HIGH_JUNCTION;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_MED_JUNCTION;
import static org.firstinspires.ftc.teamcode.Components.LiftArm.liftArmStates.ARM_INTAKE;
import static org.firstinspires.ftc.teamcode.Components.LiftArm.liftArmStates.ARM_OUTTAKE;

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
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFGamepad;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class PwPRobot extends BasicRobot {
    private Aligner aligner = null;
    private Claw claw = null;
    private LiftArm liftArm = null;
    private ClawExtension clawExtension = null;
    private Lift lift = null;
    private RFGamepad gp = null;
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
        gp = new RFGamepad();
    }

    public void stop() {
        lift.setLiftPower(0.0);
        logger.log("/RobotLogs/GeneralRobot", "program stoped");
    }
    public void delay(double p_delay){
        queuer.addDelay(p_delay);
    }
    public void waitForFinish(int condition){
        queuer.waitForFinish(condition);
    }
    public void waitForFinish(){
        queuer.waitForFinish();
    }
    public void autoAim() {
        if (queuer.queue(false, !roadrun.isBusy())) {
            if (!roadrun.isBusy() && field.lookingAtPole()) {
                Trajectory trajectory = roadrun.trajectoryBuilder(roadrun.getPoseEstimate()).lineToLinearHeading(
                        field.getDropPosition()).build();
                roadrun.followTrajectoryAsync(trajectory);
            }
        }
    }
    public void teleAutoAim(Trajectory trajectory){
        roadrun.followTrajectoryAsync(trajectory);
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
                roadrun.followTrajectoryAsync(trajectory);
            }
        }
    }
    public void followTrajectoryAsync (Trajectory trajectory, boolean clawClosed) {
        if (queuer.queue(false, !roadrun.isBusy()||CLAW_CLOSED.getStatus())) {
            if (!roadrun.isBusy()) {
                roadrun.followTrajectoryAsync(trajectory);
            }
        }
    }
    public void setFirstLoop(boolean value) {
        queuer.setFirstLoop(value);
    }

    public void openClaw() {
        openClaw(true);
    }

    public void openClaw(boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, CLAW_OPEN.getStatus())) {
            claw.updateClawStates();
            claw.openClaw();
        }
    }

    public void closeClaw() {
        closeClaw(true);
    }
    public void closeClaw(boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, CLAW_CLOSED.getStatus())) {
            claw.updateClawStates();
            if(claw.coneDistance()<5) {
                claw.closeClaw();
            }
            else{
                logger.log("/RobotLogs/GeneralRobot","ConeDistance: " + claw.coneDistance() );
            }
        }
    }

    public void toggleClawPosition() {
        toggleClawPosition(true);
    }

    public void toggleClawPosition(boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, op.getRuntime() > claw.clawServoLastSwitchTime +
                claw.CLAW_SERVO_SWITCH_TIME)) {
            claw.toggleClawPosition();
        }
    }

    public boolean isConeReady() {
        return claw.isConeReady();
    }

    public void updateClawStates() {
        claw.updateClawStates();
    }

    public void liftToPosition(Lift.LiftConstants targetJunction){
        if (queuer.queue(true, lift.isDone())){
            lift.liftToPosition(targetJunction);
        }
    }
    public void liftToTargetAuto(){
        lift.liftToTargetAuto();
    }
    public void liftToPosition(int tickTarget) {
        if (queuer.queue(true, lift.isDone())) {
            lift.liftToPosition(tickTarget);
        }
    }
    public void liftToPosition(int tickTarget, boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, lift.isDone())) {
            lift.liftToPosition(tickTarget);
        }
    }
    public void setLiftPower(double p_power) {
        lift.setLiftPower(p_power);
    }

    public void lowerLiftArmToIntake() {
        lowerLiftArmToIntake(true);
    }

    public void lowerLiftArmToIntake(boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, ARM_INTAKE.getStatus())) {

            liftArm.updateLiftArmStates();
            liftArm.lowerLiftArmToIntake();
        }
    }

    public void raiseLiftArmToOuttake() {
        raiseLiftArmToOuttake(true);
    }

    public void raiseLiftArmToOuttake(boolean p_asynchronous) {
        if (queuer.queue(p_asynchronous, ARM_OUTTAKE.getStatus())) {

            liftArm.updateLiftArmStates();
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
    public void liftToTarget(){
        lift.liftToTarget();
    }
    public void setLiftTarget(double p_target){
        lift.setLiftTarget(p_target);
    }
    public void setLiftVelocity(double velocity){
        lift.setLiftVelocity(velocity);
    }
    public void teleOp() {

        //omnidirectional movement + turning

        if(op.gamepad2.y){
            lift.setLiftTarget(LIFT_HIGH_JUNCTION.getValue());
        }
        if(op.gamepad2.b){
            lift.setLiftTarget(LIFT_MED_JUNCTION.getValue());
        }
        if(op.gamepad2.a){
            liftArm.lowerLiftArmToIntake();
            lift.setLiftTarget(0);
        }

        if(op.gamepad1.dpad_left&&op.gamepad2.dpad_left){
            lift.resetEncoder();
        }
        //manual lift up/down
        if(op.gamepad1.dpad_down&&op.gamepad2.dpad_down){
            lift.setLiftRawPower((op.gamepad2.right_trigger - op.gamepad2.left_trigger)/3);
        }
        else if (op.gamepad2.right_trigger > 0.1 || op.gamepad2.left_trigger > 0.1) {
            lift.setLiftPower((op.gamepad2.right_trigger - op.gamepad2.left_trigger));
        }
        //when not manual lifting, automate lifting
        else {
//            lift.setLiftPower(0);
            lift.liftToTarget();
        }

        if (field.lookingAtPole()&&op.gamepad1.dpad_up && !roadrun.isBusy()) {
            field.updateTrajectory();
            teleAutoAim(field.getTrajectory());
        }
        else if(roadrun.isBusy()){
            //nothin
        }
        else {
            roadrun.setWeightedDrivePower(
                    new Pose2d(
                            -op.gamepad1.left_stick_y*0.7,
                            -op.gamepad1.left_stick_x,
                            -op.gamepad1.right_stick_x*0.8
                    )
            );
            logger.log("/RobotLogs/GeneralRobot", "Mecanum,setWeightedDriverPower(Pose2d),Set driving to FWD/BWD | Strafe | Angle " + -op.gamepad1.left_stick_y*0.7 + " | " + -op.gamepad1.left_stick_x + " | " + -op.gamepad1.right_stick_x*0.8);

        }
//        //toggle automate lift target to higher junc
//        if (op.gamepad2.dpad_up) {
//            lift.toggleLiftPosition(1);
//        }
//        //toggle automate lift target to lower junc
//        if (op.gamepad2.dpad_down) {
//            lift.toggleLiftPosition(-1);
//        }
        //toggle liftArm position
        if (op.gamepad2.right_bumper) {
            if(ARM_OUTTAKE.getStatus()) {
                liftArm.lowerLiftArmToIntake();

            }
            else {
                liftArm.raiseLiftArmToOuttake();
            }
        }
        if (op.gamepad1.x) {
                claw.openClaw();
        }
        claw.closeClaw();
        if(op.getRuntime()- claw.getLastTime()>.3&&op.getRuntime()- claw.getLastTime()<.5){
            liftArm.raiseLiftArmToOuttake();
        }

        //manual open/close claw (will jsut be open claw in the future)

        //will only close when detect cone
        //claw.closeClaw
        gp.readGamepad(op.gamepad2.y, "gamepad1_y", "High Junction");
        gp.readGamepad(op.gamepad1.x, "gamepad1_x", "Toggle Claw Open/Close");
        gp.readGamepad(op.gamepad2.a, "gamepad1_a", "Ground Junction");
        gp.readGamepad(op.gamepad2.b, "gamepad1_b", "Medium Junction");
        gp.readGamepad(op.gamepad1.left_stick_y, "gamepad1_left_stick_y", "Forwards/Backwards");
        gp.readGamepad(op.gamepad1.left_stick_x, "gamepad1_left_stick_x", "Left/Right");
        gp.readGamepad(op.gamepad1.right_stick_x, "gamepad1_right_stick_x", "Turn Angle Left/Right");
        gp.readGamepad(op.gamepad2.left_trigger, "gamepad2_left_trigger", "Lift going down power");
        gp.readGamepad(op.gamepad2.right_trigger, "gamepad2_right_trigger", "Lift going up power");
        gp.readGamepad(op.gamepad2.right_bumper, "gamepad2_right_bumper", "Lift Arm Toggle Up/Down");

        roadrun.update();
        liftArm.updateLiftArmStates();
        claw.updateClawStates();
        lift.updateLiftStates();
    }
}
