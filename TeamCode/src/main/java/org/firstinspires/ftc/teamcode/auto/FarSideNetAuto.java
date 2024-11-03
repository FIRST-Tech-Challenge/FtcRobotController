package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.DriverHubHelp;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@Autonomous(name="FarSideNet Auto")
public class FarSideNetAuto extends LinearOpMode {
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

        waitForStart();
        //wait for other team
        sleep(5000);

        //go forward
        strafe = 0;
        rotate = 0;
        forward = 0.4;
        robot.drive(forward,strafe,rotate);

        sleep(2400);

        //go back
        strafe = 0;
        rotate = 0;
        forward = -0.2;
        robot.drive(forward,strafe,rotate);


        sleep(1000);

        //start by changing this and going back
        strafe = 0;
        rotate = -0.5;
        forward = 0;
        robot.drive(forward,strafe,rotate);

        sleep(800);


        strafe = 0;
        rotate = 0;
        forward = 0.4;
        robot.drive(forward,strafe,rotate);


        sleep(1800);

        strafe = 0;
        rotate = 0.5;
        forward = 0;
        robot.drive(forward,strafe,rotate);


        sleep(800);
        strafe = 0;
        rotate = 0;
        forward = -0.4;
        robot.drive(forward,strafe,rotate);


        sleep(2000);


        robot.drive(0,0,0);

        liftPower = -0.5;
        lift.moveLift(liftPower);
        sleep(1500);
        liftPower = 0;
        lift.moveLift(liftPower);

        armPosition = 0.5;
        arm.setPosition(armPosition);
        sleep(10000);


        while(opModeIsActive())
        {


        }
    }
}
