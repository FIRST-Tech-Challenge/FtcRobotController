package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.LedLights;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.TensorflowVision;

import org.firstinspires.ftc.teamcode.yise.Parameters;

/**
 * AutonomousConfig
 * EndingPosition
 * Color
 */
@Autonomous(name="Autonomous", group="Linear Opmode")
public class Auto extends LinearOpMode {
    //Initialize timer
    private ElapsedTime runtime = new ElapsedTime();
    int Prop;

    public double startX = 0;

    public double startY = 0;

    public double startZ = 0;

    public double propX = 0;

    public double propY = 0;

    public double propZ = 0;

    public double multiplier = 1;




    public void RedExterior() {

    }
    public void RedInterior() {

    }
    public void BlueExterior() {

    }
    public void BlueInterior() {

    }



    @Override
    public void runOpMode() {
        //Initialize RR
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        LiftArm arm = new LiftArm(hardwareMap);

        TensorflowVision vision = new TensorflowVision(hardwareMap);

        LedLights leds = new LedLights(hardwareMap);

        double time = getRuntime();

        Parameters parameter = new Parameters();


        //Sense cones
        while (!isStarted()) {

            sleep(2000);

            Prop = vision.getPropPosition();

            leds.setLed(LedLights.ledStates.DARK);

            telemetry.addData("Prop: ", Prop);
            telemetry.addData("Time:", getRuntime());
            telemetry.update();
        }

        if (isStopRequested()) return;

        leds.setLed(LedLights.ledStates.RED);

        /**
         *Bot starting position & variables
         */

        if (Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
            startX = -41.5;
        } else if (Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
            startX = 17.5;
        }

        if (Parameters.allianceColor == Parameters.Color.BLUE) {
            startY = 62;
            startZ = 90;
            multiplier = 1;
            leds.setLed(LedLights.ledStates.BLUE);
        } else if (Parameters.allianceColor == Parameters.Color.RED) {
            startY = -61;
            startZ = -90;
            multiplier = -1;
            leds.setLed(LedLights.ledStates.RED);
        }

            Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startZ));
            drive.setPoseEstimate(startPose);

        if(vision.getPropPosition() == 2) {
            if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
                propY = 0;
                propZ = 0;
                propX = 0;
            } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
                propY = 0;
                propZ = 0;
                propX = 0;
            } else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
                propY = 0;
                propZ = 0;
                propX = 0;
            } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
                propY = 0;
                propZ = 0;
                propX = 0;
            }
        } else if (vision.getPropPosition() == 1) {

        } else if (vision.getPropPosition() == 0) {

        }


        /**
         * Trajectory sequences that contain driving instructions
         */


        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(propX, propY, Math.toRadians(propZ)))
                .back(9)
                .back(-6)
                .waitSeconds(2)
                .build();

        TrajectorySequence traj1_2 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(16, -37, Math.toRadians(-90)))
                .back(-5)
                .waitSeconds(2)
                .build();

        TrajectorySequence traj1_3 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(24, -43, Math.toRadians(-90)))
                .back(-3)
                .waitSeconds(2)
                .build();


        TrajectorySequence traj2_1 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(38, -45, Math.toRadians(180)))
                .waitSeconds(2)
                .build();

        TrajectorySequence traj2_3 = drive.trajectorySequenceBuilder(traj1_3.end())
                .back(-2)
                .lineToLinearHeading(new Pose2d(38, -68, Math.toRadians(180)))
                .waitSeconds(2)
                .build();


        TrajectorySequence traj2_2 = drive.trajectorySequenceBuilder(traj1_2.end())
                .lineToLinearHeading(new Pose2d(38, -68, Math.toRadians(180)))
                .waitSeconds(2)
                .build();

        TrajectorySequence traj3_1 = drive.trajectorySequenceBuilder(traj2_1.end())
                .lineToLinearHeading(new Pose2d(47, -28, Math.toRadians(180)))
                .addDisplacementMarker(10,() -> {
                    arm.extendAndDrop(LiftArm.Distance.AUTO);
                })

                .build();
        TrajectorySequence traj4_1 = drive.trajectorySequenceBuilder(traj3_1.end())
                .forward(-5)
                .addDisplacementMarker(5,arm::openTrapdoor)
                .forward(10)
                .addDisplacementMarker(20, () -> {
                    arm.retract();
                    arm.closeTrapdoor();
                })
                .strafeLeft(28)
                .forward(-22)
                .build();


        TrajectorySequence traj3_2 = drive.trajectorySequenceBuilder(new Pose2d(38, -64, Math.toRadians(180)))
                .addDisplacementMarker(10, () -> {
                    arm.extendAndDrop(LiftArm.Distance.AUTO);
                })
                .lineToLinearHeading(new Pose2d(47, -34, Math.toRadians(180)))
                .build();
       TrajectorySequence traj4_2 = drive.trajectorySequenceBuilder(traj3_2.end())
               .forward(-5)
               .addDisplacementMarker(5, arm::openTrapdoor)
               .forward(10)
               .addDisplacementMarker(20,() -> {
                   arm.retract();
                   arm.closeTrapdoor();
               })
               .strafeLeft(28)
               .forward(-18)
               .build();

        TrajectorySequence traj3_3 = drive.trajectorySequenceBuilder(new Pose2d(38, -64, Math.toRadians(180)))
                .addDisplacementMarker(() ->{
                    arm.extendAndDrop(LiftArm.Distance.AUTO);
                })
                .lineToLinearHeading(new Pose2d(46, -40, Math.toRadians(180)))
                .forward(-4)
                .waitSeconds(2)
                .build();
        TrajectorySequence traj4_3 = drive.trajectorySequenceBuilder(traj3_3.end())
                .addDisplacementMarker(arm::openTrapdoor)
                .forward(4)
                .waitSeconds(.6)
                .build();

        TrajectorySequence traj5_3 = drive.trajectorySequenceBuilder(traj4_3.end())
               .addDisplacementMarker(8,() -> {
                   arm.retract();
                   arm.closeTrapdoor();
               })
                .forward(6)
                .strafeLeft(28)
                .forward(-18)
                .build();
        TrajectorySequence test = drive.trajectorySequenceBuilder(startPose)
                .back(40)
                .addDisplacementMarker(2, () -> {
                    arm.retract();
                    arm.extendAndDrop(LiftArm.Distance.AUTO);
                    arm.openTrapdoor();
                })
                .waitSeconds(15)
                .build();



        //Follow trajectories in order
        //switch between parking
        if(vision.getPropPosition() == 2) {
            //drive.followTrajectorySequence(test);
            drive.followTrajectorySequence(traj1);
            drive.followTrajectorySequence(traj2_1);
            drive.followTrajectorySequence(traj3_1);
            drive.followTrajectorySequence(traj4_1);

        } else if (vision.getPropPosition() == 1) {
            //drive.followTrajectorySequence(test);
            drive.followTrajectorySequence(traj1_2);
            drive.followTrajectorySequence(traj2_2);
            drive.followTrajectorySequence(traj3_2);
            drive.followTrajectorySequence(traj4_2);

        } else if (vision.getPropPosition() == 0) {
            //drive.followTrajectorySequence(test);
            drive.followTrajectorySequence(traj1_3);
            drive.followTrajectorySequence(traj2_3);
            drive.followTrajectorySequence(traj3_3);
            drive.followTrajectorySequence(traj4_3);
            drive.followTrajectorySequence(traj5_3);
        }
    }
}
