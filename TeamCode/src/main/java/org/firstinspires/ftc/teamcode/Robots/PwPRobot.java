package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Aligner;
import org.firstinspires.ftc.teamcode.Components.CV.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.ClawExtension;
import org.firstinspires.ftc.teamcode.Components.Field;
import org.firstinspires.ftc.teamcode.Components.Lift;
import org.firstinspires.ftc.teamcode.Components.StateMachine;
import org.firstinspires.ftc.teamcode.Old.Components.Misc.ColorDistanceRevV3;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class PwPRobot extends BasicRobot{
    private Aligner aligner =null;
    private ColorDistanceRevV3 alignerSensor;
    private Claw claw = null;
    private ClawExtension clawExtension = null;
    private Lift lift = null;
    public CVMaster cv= null;
    public Field field = null;
    public SampleMecanumDrive roadrun = null;
    public static StateMachine states = null;



    public PwPRobot(LinearOpMode opMode, boolean p_isTeleop) {
        super(opMode, p_isTeleop);
        states = new StateMachine();
        roadrun = new SampleMecanumDrive(op.hardwareMap);
        field = new Field(roadrun);
        aligner = new Aligner();
        alignerSensor = new ColorDistanceRevV3();
        claw = new Claw();
        clawExtension = new ClawExtension();
        lift = new Lift();
        cv = new CVMaster(roadrun, field);
    }
    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        if (queuer.queue(false, !roadrun.isBusy())) {
            if (!roadrun.isBusy()) {
                roadrun.followTrajectorySequenceAsync(trajectorySequence);
            }
        }
    }
    public void setFirstLoop(boolean value){
        queuer.setFirstLoop(value);
    }

    public void openClaw() {
        if (queuer.queue(true, op.getRuntime() > claw.clawServoLastSwitchTime +
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

    public void spinAlignerIntake() {
        if (queuer.queue(true, alignerSensor.getSensorDistance() <
                aligner.CONE_IN_ALIGNER_DISTANCE)) {

            aligner.spinAlignerIntake();
        }
    }

    public void stopAlignerIntake() {
        if (queuer.queue(true, op.getRuntime() > aligner.alignerMotorLastStopTime +
                aligner.ALIGNER_MOTOR_STOP_TIME)) {

            aligner.stopAlignerIntake();
        }
    }

    public void reverseAlignerIntake() {
        if (queuer.queue(true, alignerSensor.getSensorDistance() >
                aligner.CONE_OUT_OF_ALIGNER_DISTANCE)) {

            aligner.reverseAlignerIntake();
        }
    }
}
