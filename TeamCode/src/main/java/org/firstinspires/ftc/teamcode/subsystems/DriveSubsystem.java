package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.ejml.dense.row.mult.VectorVectorMult_CDRM;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.List;
import java.util.Vector;
import java.util.concurrent.TimeUnit;

/**
 * A subsystem that uses the {@link SampleMecanumDrive} class.
 * This periodically calls {@link SampleMecanumDrive#update()} which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class DriveSubsystem extends SubsystemBase {

    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    SampleMecanumDrive m_drive;
    double m_allianceHeadingOffset;
    double THROTTLEMINLEVEL = 0.4;
    boolean m_isPotentialFieldEn = false;
    double extHeading;
    Pose2d pose;
    List<Double> wheelPositions;
//    Timing.Timer m_timer;
//    Long prev_time;


    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry, Pose2d initialPose, double allianceHeadingOffset)
    {
        m_hardwareMap=hardwareMap;
        m_telemetry=telemetry;
        m_drive=new SampleMecanumDrive(m_hardwareMap, m_telemetry);
        m_drive.setPoseEstimate(initialPose);
        m_allianceHeadingOffset = allianceHeadingOffset;
//        m_timer = new Timing.Timer(300, TimeUnit.MILLISECONDS);
//        m_timer.start();
//        prev_time = m_timer.elapsedTime();
    }

    public void drive(double leftX, double leftY, double rightX, double throttle, boolean isFieldCentric)
    {
        Pose2d poseEstimate = getPoseEstimate();
        Vector2d input_vec = new Vector2d(leftY, leftX).rotated(
                isFieldCentric ? -(poseEstimate.getHeading() - Math.toRadians(m_allianceHeadingOffset)) :0
        );

        Vector2d CorrectedInput = input_vec;


        double throttleSlope = 1 - THROTTLEMINLEVEL;
        double throttleScale = throttleSlope * throttle + THROTTLEMINLEVEL;
        m_drive.setWeightedDrivePower(
                new Pose2d(
                        CorrectedInput.getX() * throttleScale,
                        CorrectedInput.getY() * throttleScale,
                        rightX * throttleScale
                )
        );
    }

    private Vector2d quadraticControlLaw(Vector2d inputVec)
    {
        double outX = inputVec.getX() * inputVec.getX();
        double outY = inputVec.getY() * inputVec.getY();
        return new Vector2d(outX, outY);
    }

    public void TogglePotentialFields()
    {
        m_isPotentialFieldEn = !m_isPotentialFieldEn;
    }



    public void correctHeadingOffset()
    {
        m_allianceHeadingOffset = Math.toDegrees(getPoseEstimate().getHeading());
    }

    public void update()
    {
//        m_telemetry.addData("timestep: ", m_timer.elapsedTime() - prev_time);
//        m_telemetry.update();
        m_drive.update();
//        prev_time = m_timer.elapsedTime();
    }

    public boolean isBusy() {
        return m_drive.isBusy();
    }

    public void stop(){
        drive(0,0,0, 0, true);
    }

    public void followTrajectory(Trajectory trajectory){
        m_drive.followTrajectory(trajectory);
    }

    public void followTrajectoryAsync(Trajectory trajectory) { m_drive.followTrajectoryAsync(trajectory);}

    public void turnAsync(double angle) {m_drive.turnAsync(angle);}

    public void setPoseEstimate(Pose2d pose)
    {
        m_drive.setPoseEstimate(pose);
    }

    public Pose2d getPoseEstimate()
    {
        return m_drive.getPoseEstimate();
    }


    public void updatePoseEstimate()
    {
        m_drive.updatePoseEstimate();
    }

//    public void IMUCorrectedHeading(){
//        pose = getPoseEstimate();
//        extHeading = Angle.norm(m_drive.getExternalHeading()- pose.getHeading());
//        pose = new Pose2d(getPoseEstimate().vec(), extHeading);
//        setPoseEstimate(pose);
//    }

    public TrajectoryFollowerCommand runTrajectory(String trajectory){return new TrajectoryFollowerCommand(this, trajectory);}
    @Override
    public void periodic() {
        m_telemetry.addData("robotPosePoseEstimate", getPoseEstimate());
        m_telemetry.update();
        m_drive.updatePoseEstimate();

        //    update();
//        IMUCorrectedHeading();
//        m_telemetry.addData("Real Heading", m_drive.getRawExternalHeading());
//        m_telemetry.update();
//        wheelPositions = m_drive.getWheelPositions();
//        m_telemetry.addData("0: ", wheelPositions.get(0));
//        m_telemetry.addData("1: ", wheelPositions.get(1));
//        m_telemetry.addData("2: ", wheelPositions.get(2));
//        m_telemetry.addData("3: ", wheelPositions.get(3));
////        m_telemetry.addData("Potential Fields Enabled: ", m_isPotentialFieldEn);
//        m_telemetry.update();
    }

}
