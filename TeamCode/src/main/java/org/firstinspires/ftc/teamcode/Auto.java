package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.PoseStorage;
import org.firstinspires.ftc.teamcode.yise.TensorflowVision;

import org.firstinspires.ftc.teamcode.yise.Parameters;


@Autonomous(name="Autonomous", group="Linear Opmode")
public class Auto extends LinearOpMode {
    //Initialize timer
    private ElapsedTime runtime = new ElapsedTime();
    int prop;

    double startX = 0;
    double startZ = 0;
    double startY = 0;

    public enum Prop {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static Prop location;

    public Prop propDetection(TensorflowVision vision) {
        if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
            if (prop == 2) {
                location = Prop.RIGHT;
            } else if (prop == 1) {
                location = Prop.MIDDLE;
            } else if (prop == 0) {
                location = Prop.LEFT;
            }
        } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
            if (prop == 2) {
                location = Prop.RIGHT;
            } else if (prop == 1) {
                location = Prop.MIDDLE;
            } else if (prop == 0) {
                location = Prop.LEFT;
            }
        } else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
            if (prop == 2) {
                location = Prop.LEFT;
            } else if (prop == 1) {
                location = Prop.MIDDLE;
            } else if (prop == 0) {
                location = Prop.RIGHT;
            }
        } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
            if (prop == 2) {
                location = Prop.LEFT;
            } else if (prop == 1) {
                location = Prop.MIDDLE;
            } else if (prop == 0) {
                location = Prop.RIGHT;
            }
        }
        return location;
    }

    public TrajectorySequence propPlacment(SampleMecanumDrive drive, Pose2d startPose) {
        double z_coordinate = 0.0;
        double x_coordinate = 0.0;
        double y_coordinate = 0.0;

        switch (Parameters.allianceColor) {
            case RED:
                z_coordinate = -90;
                if (location == Prop.LEFT) {
                    y_coordinate = -30.0;
                } else if (location == Prop.MIDDLE) {
                    y_coordinate = -24.5;
                } else if (location == Prop.RIGHT) {
                    y_coordinate = -30.0;
                }
                break;
            case BLUE:
                z_coordinate = 90;
                if (location == Prop.LEFT) {
                    y_coordinate = 30.0;
                } else if (location == Prop.MIDDLE) {
                    y_coordinate = 24.5;
                } else if (location == Prop.RIGHT) {
                    y_coordinate = 30.0;
                }
        }

        switch (Parameters.autoConfig) {
            case INTERIOR:
                if (location == Prop.LEFT) {
                    x_coordinate = 22.5;
                } else if (location == Prop.MIDDLE) {
                    x_coordinate = 12.0;
                } else if (location == Prop.RIGHT) {
                    x_coordinate = 0.5;
                }
                break;
            case EXTERIOR:
                if (location == Prop.LEFT) {
                    x_coordinate = -22.5;
                } else if (location == Prop.MIDDLE) {
                    x_coordinate = -12.0;
                } else if (location == Prop.RIGHT) {
                    x_coordinate = -0.5;
                }
        }


        TrajectorySequence mySequence = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x_coordinate, y_coordinate, Math.toRadians(z_coordinate)))
                //drop purple pixel here
                .build();

        return mySequence;
    }

    public TrajectorySequence yellowPixle(SampleMecanumDrive drive, Pose2d startPose) {
        Double x_coordinate = 48.0;
        Double y_coordinate = 0.0;
        Double z_coordinate = 0.0;

        Double white_x = 0.0;
        Double white_y = 0.0;
        Double white_z = 0.0;

        if (Parameters.allianceColor == Parameters.Color.RED) {
            if (location == Prop.LEFT) {
                y_coordinate = 42.0;
            } else if (location == Prop.MIDDLE) {
                y_coordinate = 34.0;
            } else if (location == Prop.RIGHT) {
                y_coordinate = 26.0;
            }

            white_x = -55.0;
            white_y = -12.0;
            white_z = 90.0;
        } else {
            if (location == Prop.RIGHT) {
                y_coordinate = -42.0;
            } else if (location == Prop.MIDDLE) {
                y_coordinate = -34.0;
            } else if (location == Prop.LEFT) {
                y_coordinate = -26.0;
            }

            white_x = -55.0;
            white_y = 12.0;
            white_z = 90.0;
        }

        TrajectorySequence backdrop = drive.trajectorySequenceBuilder(startPose)
                .back(12)
                .lineToLinearHeading(new Pose2d(x_coordinate, y_coordinate, Math.toRadians(z_coordinate)))
                .build();

        TrajectorySequence whitestack = drive.trajectorySequenceBuilder(startPose)
                .back(12)
                .splineTo(new Vector2d(white_x, white_y), Math.toRadians(white_z))
                .forward(5)
                .waitSeconds(2) // insert intake here
                .back(50)
                .splineTo(new Vector2d(x_coordinate, y_coordinate), Math.toRadians(z_coordinate))
                .build();
        if (Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
            return backdrop;
        } else {
            return whitestack;
        }
    }


    @Override
    public void runOpMode() {

        //Initialize RR
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        LiftArm arm = new LiftArm(hardwareMap);

        TensorflowVision vision = new TensorflowVision(hardwareMap);

        Parameters parameters = new Parameters();

        //Sense cones
        while (!isStarted()) {
            propDetection(vision);

            telemetry.addData("Prop: ", prop);
            telemetry.update();
        }

        if (isStopRequested()) return;


        //Bot starting position
        if (Parameters.allianceColor == Parameters.Color.BLUE){
            startY = 62;
            startZ = 90;
        } else {
            startY = -62;
            startZ = -90;
        }

        if (Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR){
            startX = 15;
        } else {
            startX = -40;
        }

        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startZ));
        drive.setPoseEstimate(startPose);

        //Trajectory sequences contain driving instructions

        //TrajectorySequence dropPixelLeft = drive.trajectorySequenceBuilder(startPose)
        //        .lineToLinearHeading(new Pose2d(-33, 44, Math.toRadians(130)))
        //        .build();
        TrajectorySequence sequence1 = propPlacment(drive, startPose);

        TrajectorySequence sequence2 = yellowPixle(drive, sequence1.end());


        //Follow trajectories in order
        //switch between parking
        drive.followTrajectorySequence(sequence1);
        drive.followTrajectorySequence(sequence2);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
