package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PIDs.PIDController;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetection;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class PP_Auto_Quad3 extends PowerPlay_AprilTagDetection
{

    private DcMotorEx intakeMotor, liftMotorLeft, liftMotorRight;
    private Servo armJoint, clawJoint, wristJoint;
    ElapsedTime timer = new ElapsedTime();

    //Declaring lift motor powers
    double power;

    // Declaring our motor PID for the lift; passing through our PID values
    PIDController motorPID = new PIDController(0,0,0, timer); // This is where we tune PID values

    public PP_Auto_Quad3(){
        super.runOpMode();

        waitForStart();

        // Motors \\
        liftMotorLeft = (DcMotorEx)hardwareMap.dcMotor.get("liftMotorLeft");
        liftMotorRight = (DcMotorEx)hardwareMap.dcMotor.get("liftMotorRight");

        liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // RUN_USING_ENCODER limits motors to about 80% of their full potential
        liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos \\
        armJoint = hardwareMap.get(Servo.class, "armJoint");
        clawJoint = hardwareMap.get(Servo.class, "clawJoint");
        wristJoint = hardwareMap.get(Servo.class, "wristJoint");

    }

    @Override
    public void runOpMode()
    {
        // bot object created
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);

        bot = runTrajectories(bot);

        // LOOPS TO RUN ASYNC \\
        while (opModeIsActive()){
            bot.update();

            // Our motors will stop running exactly at position 100
            power = motorPID.update(100, intakeMotor.getCurrentPosition());
            liftMotorLeft.setPower(power);
            liftMotorRight.setPower(power);
        }
    }// end of runOpMode()

    public SampleMecanumDrive runTrajectories(SampleMecanumDrive bot){
        Pose2d startPose = new Pose2d(-35, 61.8, Math.toRadians(270)); // x, y, heading (angle in radians)
        bot.setPoseEstimate(startPose);

        // TRAJECTORY SEQUENCES \\
        Pose2d endPose = cycle(bot, startPose);

        if(tagUse == 1) {
            TrajectorySequence sussyBaka = bot.trajectorySequenceBuilder(endPose)
                    .lineToLinearHeading(new Pose2d(-12.2, 15.5, Math.toRadians(-109)))
                    .build();
            bot.followTrajectorySequenceAsync(sussyBaka);
            telemetry.addData("Chris Pratt","Is Currently In The Mushroom Kingdom");
        }else if(tagUse ==2) {
            telemetry.addData("Walter White","Curently Has No Pants");
        }else{
            TrajectorySequence jacobIsCute = bot.trajectorySequenceBuilder(endPose)
                    .lineTo(new Vector2d(-58.2,24.6))
                    .build();

            bot.followTrajectorySequenceAsync(jacobIsCute);
            telemetry.addData("Bohan and Abhilash"," = Very Cute");
        }

        return bot;
    }

    public Pose2d cycle(SampleMecanumDrive bot, Pose2d currentPosition){
        TrajectorySequence openingMove =  bot.trajectorySequenceBuilder(currentPosition)
            .lineToLinearHeading(new Pose2d(-33,8,Math.toRadians(-39.75))) // Drive to cone stack

            // TEMPORAL MARKER TO INTAKE

            .lineToLinearHeading(new Pose2d(-57, 12.3, Math.toRadians(0)))
            .waitSeconds(2.5) //This would be replaced with an actual intake function
            .lineToLinearHeading(new Pose2d(-33, 8, Math.toRadians(-39.75)))
            .waitSeconds(1)//Deposit at junction; Under the impression that using the async PID, the slides will be already be moved up
            .build();

        Pose2d endPose = new Pose2d(-33,8,Math.toRadians(-39.75)); // Deposit at junction

        TrajectorySequence cycles =  bot.trajectorySequenceBuilder(currentPosition)
                .lineToLinearHeading(new Pose2d(-57, 12.3, Math.toRadians(0))) // back to the cone stack
                .waitSeconds(2.5) //This would be replaced with an actual intake function
                .lineToLinearHeading(new Pose2d(-33, 8, Math.toRadians(-39.75))) // go to junction
                .waitSeconds(1)//Under the impression that using the async PID, the slides will be already be moved up
                .build();

        bot.followTrajectorySequenceAsync(openingMove);
        for(int i = 1; i <= 3; i++){
          bot.followTrajectorySequenceAsync(cycles);
         }
        return endPose;
    }
}
