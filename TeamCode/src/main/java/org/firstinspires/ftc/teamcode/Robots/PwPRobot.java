package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Aligner;
import org.firstinspires.ftc.teamcode.Components.CV.CVMaster;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.ClawExtension;
import org.firstinspires.ftc.teamcode.Components.Field;
import org.firstinspires.ftc.teamcode.Components.Lift;
import org.firstinspires.ftc.teamcode.Components.StateMachine;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class PwPRobot extends BasicRobot{
    private Aligner aligner =null;
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
}
