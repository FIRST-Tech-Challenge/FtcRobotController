package org.firstinspires.ftc.teamcode.AutoTest.Roadrunner;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name="LeftSideAuto", group="org.firstinspires.ftc.teamcode.AutoTest.Roadrunner")
@Config
public class LeftSideAuto extends LinearOpMode {

    RobotActionConfig value = new RobotActionConfig();
    RobotHardware robot = new RobotHardware();
    static final double COUNTS_PER_MOTOR_GOBILDA_435    = 384.5;
    static final double COUNTS_PER_MOTOR_GOBILDA_312    = 537.7;
    static final double DRIVE_GEAR_REDUCTION            = 1.5; //24:16 Motor:Wheel
    static final double WHEEL_DIAMETER_MM               = 96; // Wheel diameter mm
    static final double COUNTS_PER_MM_Drive             = (COUNTS_PER_MOTOR_GOBILDA_435 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);
    static final double COUNTS_PER_CM_Slides = COUNTS_PER_MOTOR_GOBILDA_312 / 38.2; //Ticks Per Rotation * Pulley Circumference

    //
    public static double intake_slide_Extension = 0.6;// range(0.3 - 0.65)
    public static double intake_slide_Retract   = 0.3;

    public static double intake_Rotation        = 0.49;

    public static double intake_Arm_initial     = 0.35;//0-0.56
    public static double intake_Arm_down        = 0.05;
    public static double intake_Arm_retract     = 0.53;

    public static double intake_Claw_Open       = 0.55;
    public static double intake_Claw_Close      = 0.3;

    //Deposit Config
    public static int deposit_Slide_down_Pos         = 50;   //slides Position Configure
    public static int deposit_Slide_Highbar_Pos      = 795;  //slides Position Configure
    public static int deposit_Slide_Highbasket_Pos   = 2800; //slides Position Configure

    public static double deposit_Wrist_dump_Pos         = 0.3;
    public static double deposit_Wrist_retract_Pos      = 0.1;

    public static double deposit_Arm_dump_Pos           = 0.8;
    public static double deposit_Arm_retract_Pos        = 0.0;

    public static double deposit_Arm_hook_Pos           = 0.8;
    public static double deposit_Claw_Open              = 0.11;
    public static double deposit_Claw_Close             = 0.0;

    public static double dumpTime                       = 1.8;
    public static double retractTime                    = 3.2;

    public static double deposit_Slide_UpLiftPower      = 0.9;  //slides power
    public static double downLiftPower                  = 0.3;  //slides power

    private ElapsedTime intakeTimer = new ElapsedTime();


    //Timer
    static ElapsedTime hook_Time = new ElapsedTime();
    static double intake_Wait_Time = 0.5;
    static double deposit_Wait_Time = 0.5;

    //Segment 1 Distance
    static double first_forward = -500;
    static double speed = 0.2*2.25;
    //movement positions
    public static double basket_x_coordinate = -18;
    public static double basket_y_coordinate = -9;
    public static double first_sample_x_coordinate = -6;
    public static double first_sample_y_coordinate = 25;
    public static double second_sample_x_coordinate = -20;
    public static double second_sample_y_coordinate = 25;
    public static double third_sample_x_coordinate = -20;
    public static double third_sample_y_coordinate = 25;
    public static double third_sample_heading = 120;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(basket_x_coordinate, basket_y_coordinate,Math.toRadians(-45)))
                .addDisplacementMarker(7,() -> {
                    Slides_Move(3226,0.8);
                })
                .lineToLinearHeading(new Pose2d(first_sample_x_coordinate, first_sample_y_coordinate,Math.toRadians(-90)))
                //.addDisplacementMarker(() -> {})
                .waitSeconds(3)
                .lineToLinearHeading(new Pose2d(basket_x_coordinate, basket_y_coordinate,Math.toRadians(-45)))
                //.addDisplacementMarker(() -> {})
                .addDisplacementMarker(5,() -> {
                    Slides_Move(3226,0.8);
                })
                .lineToLinearHeading(new Pose2d(second_sample_x_coordinate, second_sample_y_coordinate,Math.toRadians(-90)))
                //.addDisplacementMarker(() -> {})
                .waitSeconds(3)
                .lineToLinearHeading(new Pose2d(basket_x_coordinate, basket_y_coordinate,Math.toRadians(-45)))
                //.addDisplacementMarker(() -> {})
                .waitSeconds(6)
                .lineToLinearHeading(new Pose2d(third_sample_x_coordinate, third_sample_y_coordinate,Math.toRadians(third_sample_heading)))
                //.addDisplacementMarker(() -> {})
                .waitSeconds(3)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }

    private void Slides_Move(int dist, double speed) {
        robot.liftMotorLeft.setTargetPosition(dist);
        robot.liftMotorRight.setTargetPosition(dist);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
        while (opModeIsActive() && (robot.liftMotorLeft.isBusy() && robot.liftMotorRight.isBusy())) {

        }
        sleep(500);
    }

}
