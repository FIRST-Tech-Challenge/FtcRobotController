package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.visionex.LimelightLocalization;
//import org.firstinspires.ftc.teamcode.roadrunner.tuning.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.DriverHubHelp;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
//@Autonomous(name="Far Claw")
public class FarSideClawAuto extends LinearOpMode {
    private GamepadEvents controller;
    private MechDrive robot;
    private LimelightLocalization limelight;
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
        limelight = new LimelightLocalization(hardwareMap);
        imu = new Imu(hardwareMap);
        screen = new DriverHubHelp();
//        deadwheels = new ThreeDeadWheelLocalizer(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
          lift = new Lift(hardwareMap, "lift", "lift");
//        arm = new Arm(hardwareMap, "armRight", "armLeft");
        claw = new Claw(hardwareMap);
        clawPos = 0.6;
        waitForStart();
        //wait for other team
        sleep(5000);

        //set arm to middle
        claw.close();
        armPosition = 0.7;
        arm.setPosition(armPosition);

        waitForStart();
        strafe = 0.3;
        rotate = 0;
        forward = 0;
        robot.drive(forward,strafe,rotate);
        sleep(650);

        //go forward
        strafe = 0;
        rotate = 0;
        forward = -0.4;
        robot.drive(forward,strafe,rotate);


        sleep(2600);
        robot.drive(0,0,0);
        //turn robot around to bucket
        strafe = 0;
        rotate = 0.2;
        forward = 0;
        robot.drive(forward,strafe,rotate);

        sleep(800);

        robot.drive(0,0,0);
        liftPower = -0.8;
        lift.setPower(liftPower);
        sleep(2500);
        //lower lift
        liftPower = 0;
        lift.setPower(liftPower);

        //go forward
//        strafe = 0;
//        rotate = 0;
//        forward = -0.1;
//        robot.drive(forward,strafe,rotate);
//
//        sleep(500);



//        strafe = 0;
//        rotate = -0.1;
//        forward = 0;
//        robot.drive(forward,strafe,rotate);
//
//        sleep(500);
        //rotate robot to score
        strafe = 0;
        rotate = 0.2;
        forward = 0;
        robot.drive(forward,strafe,rotate);

        sleep(300);
        //forward to score
        strafe = 0;
        rotate = 0;
        forward = -0.1;
        robot.drive(forward,strafe,rotate);

        sleep(400);

        robot.drive(0,0,0);

        //score
        armPosition = 0.5;
        arm.setPosition(armPosition);
        sleep(1000);
        claw.release();

        armPosition = 0.6;
        arm.setPosition(armPosition);
        sleep(300);

        liftPower = 0.8;
        lift.setPower(liftPower);
        sleep(2000);
        liftPower = 0;
        lift.setPower(liftPower);
        // rotate back to straight position
        strafe = 0;
        rotate = -0.2;
        forward = 0;
        robot.drive(forward,strafe,rotate);

        sleep(1200);

        //go back after scoring high bucket
        strafe = 0;
        rotate = 0;
        forward = 0.3;
        robot.drive(forward,strafe,rotate);

        sleep(9000);

        robot.drive(0,0,0);

//        armPosition = 0.3;
//        arm.setPosition(armPosition);
//        sleep(1000);
//        //strafe to bars
//        strafe = 0.5;
//        rotate = 0;
//        forward = 0;
//        robot.drive(forward,strafe,rotate);
//
//        sleep(2600);
//
//        //rotate 180 to facing bar
//        strafe = 0;
//        rotate = 0.5;
//        forward = 0;
//        robot.drive(forward,strafe,rotate);
//
//        sleep(1500);
//        //drive forward to touch the bars
//        strafe = 0;
//        rotate = 0;
//        forward = -0.3;
//        robot.drive(forward,strafe,rotate);
//
//        sleep(1000);

//        robot.drive(0,0,0);
//
//        armPosition = 0.3;
//        arm.setPosition(armPosition);
//        sleep(1000);


        while(opModeIsActive())
        {


        }
    }
}
