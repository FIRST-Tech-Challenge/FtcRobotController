package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.IMU;
import org.firstinspires.ftc.teamcode.Components.LimitSwitches;
import org.firstinspires.ftc.teamcode.Components.Queuer;
import org.firstinspires.ftc.teamcode.Components.SummerMec.IntakeSlides;
import org.firstinspires.ftc.teamcode.Components.Ultrasonics;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class SummerMecRobot extends BasicRobot{
    IntakeSlides intakeSlides = null;
    public SampleMecanumDrive roadrun = null;
    private Queuer queuer = null;
    public Ultrasonics ultras = null;
    public IMU imu = null;
    public LimitSwitches touchs = null;
    public SummerMecRobot(LinearOpMode opMode){
        super(opMode);
        ultras = new Ultrasonics();
        touchs = new LimitSwitches();
        imu = new IMU();
        intakeSlides = new IntakeSlides();
        roadrun = new SampleMecanumDrive(op.hardwareMap, this);
        queuer = new Queuer();
    }
    public void extendIntakeTo(double position){
        if (queuer.queue(true, abs(position- intakeSlides.getPosition())<10)) {
                intakeSlides.extendIntakeTo(position);
        }
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
