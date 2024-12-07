package org.firstinspires.ftc.masters.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.components.DriveTrain;
import org.firstinspires.ftc.masters.components.ITDCons;
import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outake;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

@Autonomous(name="specimen Blue")
public class SpecimenBlue extends LinearOpMode {

    SampleMecanumDrive driveTrain;
    Outake outake;
    Intake intake;

    enum State {
        START,
        AT_SUBMERSIBLE,
        SUBMERSIBLE_TO_PARK,
        PARK
    }


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        driveTrain = new SampleMecanumDrive(hardwareMap);
        outake = new Outake(driveTrain.init, telemetry);
        intake = new Intake(driveTrain.init, telemetry);
        outake.closeClaw();

        Pose2d startPose= new Pose2d(new Vector2d(-8.5,54),Math.toRadians(90));
        driveTrain.setPoseEstimate(startPose);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Trajectory start= driveTrain.trajectoryBuilder(startPose,false)
                .lineTo(new Vector2d(-8.5,25))
                .build();

        Trajectory toPark = driveTrain.trajectoryBuilder(start.end(), false)
                .lineToConstantHeading(new Vector2d(-72,72))
                .build();


        State currentState = State.START;
        int target= 0;

        ElapsedTime scoreTime= null;
        ElapsedTime openClaw= null;

        outake.setPID(0.0001);

        waitForStart();

        driveTrain.followTrajectoryAsync(start);

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.update();
            outake.moveSlide(target);

            switch (currentState) {
                case START:
                    if (!driveTrain.isBusy()){
                        currentState = State.AT_SUBMERSIBLE;
                    } else {
                        outake.diffy1(ITDCons.SpecimenDiffy1);
                        outake.diffy2(ITDCons.SpecimenDiffy2);
                        target= ITDCons.SpecimenTarget+10000;
                    }
                    break;
                case AT_SUBMERSIBLE:
                    if (target>35500 && scoreTime==null){
                        outake.diffy1(ITDCons.ReleaseDiffy1);
                        outake.diffy2(ITDCons.ReleaseDiffy2);
                        target= ITDCons.ReleaseTarget;
                        scoreTime = new ElapsedTime();
                    } else if (scoreTime!=null && scoreTime.milliseconds()>1500){
                        outake.openClaw();
                        currentState= State.SUBMERSIBLE_TO_PARK;
                        driveTrain.followTrajectoryAsync(toPark);
                        outake.diffy1(ITDCons.TransDiffy1);
                        outake.diffy2(ITDCons.TransDiffy2);
                        target=0;
                    }
                    break;

            }
        telemetry.addData("pos", outake.getExtensionPos());
            telemetry.update();
        }

    }
}
