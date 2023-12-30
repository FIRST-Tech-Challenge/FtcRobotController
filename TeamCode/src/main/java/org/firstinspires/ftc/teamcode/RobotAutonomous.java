package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "SciRavens-Autonomous")
public class RobotAutonomous extends LinearOpMode {
    public Robot robot;
    public Slider slider;
    public Arm arm;
    public Claw claw;

    public AprilTag tag;
    public TgeDetection tge;
    String curAlliance = "red";
    public int zone = 2;

    private Trajectory traj2_1, traj2_2, traj2_3, traj2_4;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        slider = new Slider(robot, gamepad2);
        arm = new Arm(robot, gamepad2);
        claw = new Claw(robot, gamepad2);
        tag = new AprilTag(robot);
        tge = new TgeDetection(robot.webcam);

        traj2_1 = robot.sampleDrive.trajectoryBuilder(new Pose2d())
                .forward(27)
                .build();
        traj2_2 = robot.sampleDrive.trajectoryBuilder(new Pose2d())
                .forward(20)
                .build();
        traj2_3 = robot.sampleDrive.trajectoryBuilder(traj2_2.end())
                .strafeRight(9.0)
                .build();
        traj2_4 = robot.sampleDrive.trajectoryBuilder(traj2_3.end())
                .forward(4.0)
                .build();

        waitForStart();
        if(opModeIsActive()) {
                //zone = tge.elementDetection(telemetry);
                zone = 2;
            }
            switch(zone) {
                case 1:
                    auton1();
                    break;
                case 2:
                    auton2();
                    break;
                case 3:
                    auton3();
                    break;
            }

    }

    public void auton1() {

    }
    public void auton2() {
        claw.close_claw_right();
        claw.close_claw_left();
        robot.sampleDrive.followTrajectory(traj2_1);
        claw.open_claw_right();
        arm.arm_backdrop();
        sleep(100);
        robot.sampleDrive.turn(Math.toRadians(86.9876));
        sleep(100);
        robot.sampleDrive.followTrajectory(traj2_2);
        sleep(10000);

      /*
        robot.sampleDrive.turn(Math.toRadians(90));
        robot.sampleDrive.followTrajectory(traj2_2);
         */
    }
    public void auton3() {

    }
}

