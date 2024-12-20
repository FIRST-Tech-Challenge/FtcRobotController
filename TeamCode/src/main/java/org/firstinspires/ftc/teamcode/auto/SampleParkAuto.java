package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.MecanumDrive;
//import org.firstinspires.ftc.teamcode.roadrunner.tuning.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.utils.DriverHubHelp;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@Autonomous(name="Sample and Park Auto")
public class SampleParkAuto extends LinearOpMode {
    private GamepadEvents controller;
    private MechDrive robot;
    private Limelight limelight;
    private Imu imu;
    //    private ThreeDeadWheelLocalizer deadwheels;
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
//        deadwheels = new ThreeDeadWheelLocalizer(hardwareMap);
        lift = new Lift(hardwareMap, "liftLeft", "liftRight", "liftLeft", "liftRight");
        arm = new Arm(hardwareMap, "armRight", "armLeft");
        claw = new Claw(hardwareMap);
        clawPos = 1;

        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(12,-60,1.5708));

        waitForStart();
        claw.close();
        arm.setPosition(0);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(12,-60,1.5708))
                        .lineToY(-52)
                        .strafeTo(new Vector2d(-1,-50))
                        .turn(Math.toRadians(110))
                        .build());
        lift.setPosition(-4500);
        sleep(3000);
        arm.setPosition(0.1);
        sleep(500);
        claw.release();
        sleep(500);
        arm.setPosition(0);
        lift.setPosition(0);
//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(6,-24,1.5708))
//                        .lineToY(-45)
//                        .strafeTo(new Vector2d(110,-60))
//                        .build());
//        sleep(500);
//        arm.setPosition(0);
//        lift.setPosition(0);



        while(opModeIsActive())
        {

        }

    }
}


