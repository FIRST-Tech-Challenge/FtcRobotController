package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.DriverHubHelp;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@Autonomous(name="RedSideClaw Auto")
public class RedSideClawAuto extends LinearOpMode {
    private GamepadEvents controller;
    private MechDrive robot;
    private Limelight limelight;
    private Imu imu;
    private ThreeDeadWheelLocalizer deadwheels;
    private DriverHubHelp screen;
    double forward;
    double strafe;
    double rotate;
    private Lift lift;
    private double liftPower;
    private Arm arm;
    private double armPosition;
    private Claw claw;
    private double clawPos;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MechDrive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        imu = new Imu(hardwareMap);
        screen = new DriverHubHelp();
        deadwheels = new ThreeDeadWheelLocalizer(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        lift = new Lift(hardwareMap, "lift", "lift");
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        clawPos = 0.6;


        claw.close(clawPos);
        armPosition = 0.7;
        arm.setPosition(armPosition);

        waitForStart();

        strafe = 0;
        rotate = 0;
        forward = -0.4;
        robot.drive(forward,strafe,rotate);


        sleep(1500);

        strafe = 0;
        rotate = 0.2;
        forward = 0;
        robot.drive(forward,strafe,rotate);

        sleep(800);

        robot.drive(0,0,0);
        liftPower = -0.8;
        lift.moveLift(liftPower);
        sleep(3000);
        liftPower = 0;
        lift.moveLift(liftPower);

        strafe = 0;
        rotate = 0;
        forward = -0.1;
        robot.drive(forward,strafe,rotate);

        sleep(1000);

        strafe = 0;
        rotate = -0.1;
        forward = 0;
        robot.drive(forward,strafe,rotate);

        sleep(500);

        strafe = 0;
        rotate = 0.1;
        forward = 0;
        robot.drive(forward,strafe,rotate);

        sleep(1000);

        strafe = 0;
        rotate = 0;
        forward = 0.2;
        robot.drive(forward,strafe,rotate);

        sleep(1000);

        robot.drive(0,0,0);

        armPosition = 0.5;
        arm.setPosition(armPosition);
        sleep(4000);
        claw.release();

        robot.drive(0,0,0);
//        strafe = 0;
//        rotate = 0;
//        forward = -0.2;
//        robot.drive(forward,strafe,rotate);
//
//
//        sleep(1000);


//        strafe = 0;
//        rotate = -0.5;
//        forward = 0;
//        robot.drive(forward,strafe,rotate);
//
//        sleep(800);
//
//        strafe = 0;
//        rotate = 0;
//        forward = 0.4;
//        robot.drive(forward,strafe,rotate);
//
//
//        sleep(1800);
//
//        strafe = 0;
//        rotate = 0.5;
//        forward = 0;
//        robot.drive(forward,strafe,rotate);
//
//
//        sleep(800);
//        strafe = 0;
//        rotate = 0;
//        forward = -0.4;
//        robot.drive(forward,strafe,rotate);
//
//
//        sleep(2000);
//
//
//        robot.drive(0,0,0);
//
//        liftPower = -0.5;
//        lift.moveLift(liftPower);
//        sleep(1500);
//        liftPower = 0;
//        lift.moveLift(liftPower);
//
//        armPosition = 0.5;
//        arm.setPosition(armPosition);
//        sleep(10000);


        while(opModeIsActive())
        {


        }
    }
}

