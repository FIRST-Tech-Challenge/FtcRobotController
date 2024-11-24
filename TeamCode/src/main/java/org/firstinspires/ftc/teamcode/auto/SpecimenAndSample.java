package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.utils.DriverHubHelp;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@Autonomous(name = "Specimen and Sample Auto")
public class SpecimenAndSample extends LinearOpMode {
    private GamepadEvents controller;
    private MechDrive robot;
    private Limelight limelight;
    private Imu imu;
    private TwoDeadWheelLocalizer deadwheels;
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
//        deadwheels = new TwoDeadWheelLocalizer(hardwareMap);
        lift = new Lift(hardwareMap, "liftLeft", "liftRight", "liftLeft", "liftRight");
        arm = new Arm(hardwareMap, "armRight", "armLeft");
        claw = new Claw(hardwareMap);
        clawPos = 1;
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(-6,-60,1.5708));

        waitForStart();
        claw.close(clawPos);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-6,-60,1.5708))
                        .lineToY(-24)
                        .strafeTo(new Vector2d(0,-24))
//                .strafeTo(new Vector2d(-12,-36))
//                .turn(Math.toRadians(180))
                        .build());
        lift.setPosition(-870);
        sleep(1000);
        arm.setPosition(0.5);
        sleep(500);
        arm.setPosition(0.9);
        sleep(1000);
        claw.release();
        arm.setPosition(0.1);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(12,-60,1.5708))
                        .strafeTo(new Vector2d(-6,-24))
                        .lineToY(-60)
                        .strafeTo(new Vector2d(-18,-60))
                        .build());



        while(opModeIsActive())
        {

        }


    }
}
