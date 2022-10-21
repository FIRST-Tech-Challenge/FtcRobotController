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

    final int OPEN = 0;
    final double CLOSE = 0.5; // probably adjust later

    private DcMotorEx armMotor, liftMotorLeft, liftMotorRight;
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
        armMotor = (DcMotorEx)hardwareMap.dcMotor.get("armMotor");

        liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // RUN_USING_ENCODER limits motors to about 80% of their full potential
        liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
           /* power = motorPID.update(100, intakeMotor.getCurrentPosition());
            liftMotorLeft.setPower(power);
            liftMotorRight.setPower(power);
            */

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
            .addTemporalMarker(2.2,() -> {
                goToJunction(1000);
                armPivot(500);
                claw();
                armPivot(-500);})//temporal marker for the extake
            .lineToLinearHeading(new Pose2d(-33,8,Math.toRadians(-39.75))) // Drive to cone stack
            .addTemporalMarker(1,()->{
                goToJunction(0);
                armPivot(-950);
                claw();
                armPivot(950);})//marker for the intake, the timing will be tested so where the markers are located and times are subject to change
            .lineToLinearHeading(new Pose2d(-57, 12.3, Math.toRadians(0)))
            //.addTemporalMarker()
            .lineToLinearHeading(new Pose2d(-33, 8, Math.toRadians(-39.75)))
            .waitSeconds(1)//Deposit at junction; Under the impression that using the async PID, the slides will be already be moved up
            .build();

        Pose2d endPose = new Pose2d(-33,8,Math.toRadians(-39.75)); // Deposit at junction

        TrajectorySequence cycles =  bot.trajectorySequenceBuilder(currentPosition)
                .addTemporalMarker(3,() -> {goToJunction(0);claw();})
                .lineToLinearHeading(new Pose2d(-57, 12.3, Math.toRadians(0))) // back to the cone stack
                .lineToLinearHeading(new Pose2d(-33, 8, Math.toRadians(-39.75))) // go to junction
                .waitSeconds(1)//Under the impression that using the async PID, the slides will be already be moved up
                .build();

        bot.followTrajectorySequenceAsync(openingMove);
        for(int i = 1; i <= 3; i++){
          bot.followTrajectorySequenceAsync(cycles);
         }
        return endPose;
    }

    public void goToJunction(int target){
        double currentPosition = (liftMotorLeft.getCurrentPosition() + liftMotorRight.getCurrentPosition())/2.0; // average position

        liftMotorLeft.setTargetPosition(target);
        liftMotorRight.setTargetPosition(target);

        while(Math.abs(currentPosition - target) > 5) {
            liftMotorLeft.setPower(motorPID.update(target, currentPosition));
            liftMotorRight.setPower(motorPID.update(target, currentPosition));

            currentPosition = (liftMotorLeft.getCurrentPosition() + liftMotorRight.getCurrentPosition())/2.0; // average new position
        }
    }

    public void armPivot(int target){
        double currentPosition = armMotor.getCurrentPosition(); // average position
        armMotor.setTargetPosition(target);

        while(Math.abs(currentPosition - target) > 5) {
            armMotor.setPower(motorPID.update(target, currentPosition));
            currentPosition = armMotor.getCurrentPosition(); // average new position
        }
    }

    public void clawRotate(double increment){
        wristJoint.setPosition(wristJoint.getPosition() + increment);
    }

    public void claw(){
        if(clawJoint.getPosition() == OPEN){
            clawJoint.setPosition(CLOSE);
        }else{
            clawJoint.setPosition(OPEN);
        }
    }






}
