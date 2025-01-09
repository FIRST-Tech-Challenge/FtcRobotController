package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.vision.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.DriverHubHelp;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@Autonomous(name = "3 Sample Auto + 1 Specimen")
public class SpecimenAndSample extends LinearOpMode {
    private GamepadEvents controller;
    private MechDrive robot;
    private LimelightLocalization limelight;
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
        limelight = new LimelightLocalization(hardwareMap);
        imu = new Imu(hardwareMap);
        screen = new DriverHubHelp();
//        deadwheels = new TwoDeadWheelLocalizer(hardwareMap);
        lift = new Lift(hardwareMap, "lift", "lift");
//        arm = new Arm(hardwareMap, "armRight", "armLeft");
        claw = new Claw(hardwareMap);
        clawPos = 0.6;
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(-12,-60,1.5708));

        double specimenArmPos = 0.2;
        double offsetArm = 0.5;
        int specimenLiftPos = -1100;
//
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, -60, 1.5708))
                        .stopAndAdd(()->claw.toggle())
//                        .stopAndAdd(()->lift.setPosition(specimenLiftPos))
//                        .stopAndAdd(()-> claw.close())
//                        .stopAndAdd(()->arm.setPosition(specimenArmPos))
//                        .lineToY(-20)
////                        .waitSeconds(1000)
//                        .stopAndAdd(()->lift.setPosition(-400))
//                        .waitSeconds(2)
//                        .stopAndAdd(()-> claw.close()) //
////                        .waitSeconds(1.5) //
//                        .stopAndAdd(()->lift.setPosition(0))
//                        .stopAndAdd(()->arm.setPosition(0))
//                        .waitSeconds(0.7)
//                        .strafeTo(new Vector2d( -65, -30))
//                        .waitSeconds(0.01)
////                        .lineToX(-69)
//                        .waitSeconds(0.01)
//                        .lineToY(-30)
//                        .waitSeconds(0.01)
//                        .lineToY(-27)
//                        .waitSeconds(0.1)

//                        .waitSeconds(0.1)
//                        .stopAndAdd(()->claw.toggle())
//                        .waitSeconds(0.5)
//                        .stopAndAdd(()-> arm.setPosition(0.8))
//                        .waitSeconds(1)

//                        .stopAndAdd(()-> arm.setPosition(1))
//                        .waitSeconds(0.5)

//                        .stopAndAdd(()->claw.toggle())
//                        .waitSeconds(0.5)
//                        .stopAndAdd(()-> arm.setPosition(0))

                        //After grabbing first sample
//                        .strafeTo(new Vector2d( -79, -39))
//                        .turn(Math.toRadians(-135))
//                        .lineToY(-40)
//                        .turn(Math.toRadians(180))
//                        .waitSeconds(0.001)
//                        .lineToY(-55)
//                        .stopAndAdd(()-> lift.setPosition(-4000))
//                        .waitSeconds(3)
//                        .stopAndAdd(()->arm.setPosition(0.7))
//                        .waitSeconds(1)
//                        .stopAndAdd(()->claw.release())
//                        .stopAndAdd(()->lift.setPosition(0))
                        //after scoring first sample
                        .build());

//        lift.setPosition(specimenLiftPos);
//        sleep(1000);
//        claw.close(-1);
//        arm.setPosition(specimenArmPos - offsetArm - 0.1);
//        sleep(100);
//        arm.setPosition(specimenArmPos - offsetArm);
//        sleep(2000);
//        claw.close(0.1);
//        lift.setPosition(0);
//        //score specimen
//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(0, -24, 1.5708))
//                        //score specimen
//                        .lineToY(-39)
//                        .strafeTo(new Vector2d( -65, -39))
//                        .waitSeconds(0.001)
//                        .lineToY(-48)
//                        //grab sample
//                        .turn(Math.toRadians(135))
//
//
//                        .strafeTo(new Vector2d( -60, -60))
//                        //score first sample
//                        .strafeTo(new Vector2d( -55, -36))
//                        .turn(Math.toRadians(-135))
//                        //grab sample
//                        .turn(Math.toRadians(150))
//                        .strafeTo(new Vector2d( -60, -60))
//                        //score second sample
//                        .strafeTo(new Vector2d(-50, -24))
//                        .turn(Math.toRadians(-60))
//                        .lineToX(-60)
//                        //grab sample
//                        .lineToX(-50)
//                        .turn(Math.toRadians(60))
//                        .strafeTo(new Vector2d( -60, -60))
//                        score third sample

//                .build());
//        claw.release();
//        arm.setPosition(0.9);
//        claw.close(-1);


//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(-6,-60,1.5708))
//                        .lineToY(-24)
//                        .strafeTo(new Vector2d(0,-24))
//                .strafeTo(new Vector2d(-12,-36))
//                .turn(Math.toRadians(180))
//                        .build());
//        lift.setPosition(-870);
//        sleep(1000);
//        arm.setPosition(0.5);
//        sleep(500);
//        arm.setPosition(0.9);
//        sleep(1000);
//        claw.release();
//        arm.setPosition(0.1);
//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(12,-60,1.5708))
//                        .strafeTo(new Vector2d(-6,-24))
//                        .lineToY(-60)
//                        .strafeTo(new Vector2d(-18,-60))
//                        .build());


        while(opModeIsActive())
        {

        }


    }
}

