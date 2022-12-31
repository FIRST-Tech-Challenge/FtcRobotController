package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.SignalEdgeDetector;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class ProgressiveAuto extends PowerPlay_AprilTagDetection
{
    private boolean isAuto = true;
    SignalEdgeDetector isIntakePosition;

    // Declaring motor objects
    private Arm armControl;
    private Slide slideControl;
    private Claw clawMovement;

    public ProgressiveAuto()
    {
        super.runOpMode(); // runs the opMode of the April Tags pipeline

        waitForStart(); // waits until the start button is pressed on the DS

        // Initializing our arm, slide, and claw motorPID objects
        armControl = new Arm(hardwareMap);
        slideControl = new Slide(hardwareMap);
        clawMovement = new Claw(hardwareMap, () -> gamepad2.right_bumper, () -> gamepad2.a);
    }

    @Override
    public void runOpMode()
    {
        // BOT OBJECT CREATED
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);

       // BUILD TRAJECTORIES
        bot = buildTrajectories(bot);

       // PID AND BOT ASYNC UPDATES
        while (opModeIsActive()){
            bot.update();
            slideControl.update(telemetry);
            armControl.update(telemetry);
        }

    }// end of runOpMode()

    public SampleMecanumDrive buildTrajectories(SampleMecanumDrive bot){
        Pose2d startPose = new Pose2d(-35, 61.8, Math.toRadians(270)); // x, y, heading (angle in radians)
        bot.setPoseEstimate(startPose);

        //========== TRAJECTORY SEQUENCES ==========\\
        Pose2d endPose = cycle(bot, startPose); // cycle method -> should return a Pose2d after building and running trajectories

        // the aprilTag_ID variable is inherited from the AprilTagDetection class and represents the tagID
        if(aprilTag_ID == 1) {
            TrajectorySequence sussyBaka = bot.trajectorySequenceBuilder(endPose)
                    .lineToLinearHeading(new Pose2d(-12.2, 15.5, Math.toRadians(-109)))
                    .build();
            bot.followTrajectorySequenceAsync(sussyBaka);
            telemetry.addData("Chris Pratt","Is Currently In The Mushroom Kingdom");
        }
        else if(aprilTag_ID ==2) {
            telemetry.addData("Walter White","Curently Has No Pants");
            // bot doesn't need to move if it is in the 2nd tag zone since that is where we deposit anyway
        }
        else{
            TrajectorySequence jacobIsCute = bot.trajectorySequenceBuilder(endPose)
                    .lineTo(new Vector2d(-58.2,24.6))
                    .build();
            bot.followTrajectorySequenceAsync(jacobIsCute);
            telemetry.addData("Bohan and Abhilash"," = Very Cute");
        }
        return bot;
    }// end of runTrajectories

    public Pose2d cycle(SampleMecanumDrive bot, Pose2d currentPosition){
        TrajectorySequence openingMove =  bot.trajectorySequenceBuilder(currentPosition)
                .addTemporalMarker(2,() -> {
                    //
                    // CODE HERE
                    //
                })
                .lineToLinearHeading(new Pose2d(-33,8,Math.toRadians(-39.75))) // Drive to cone stack, 2 seconds in the previous temporal marker will activate
                .build();

        /*
        TrajectorySequence cycles =  bot.trajectorySequenceBuilder(currentPosition)
                .addTemporalMarker(3,() -> {
                    slideControl.setIntakeOrGround();
                    //slideControl.Update();
                    clawMovement.toggleOpenClose();
                })
                .lineToLinearHeading(new Pose2d(-57, 12.3, Math.toRadians(0))) // back to the cone stack
                .lineToLinearHeading(new Pose2d(-33, 8, Math.toRadians(-39.75))) // go to junction
                .waitSeconds(1)//Under the impression that using the async PID, the slides will be already be moved up
                .build();
        */

        Pose2d endPose = new Pose2d(-33,8,Math.toRadians(-39.75)); // Cycles end junction deposit

        bot.followTrajectorySequenceAsync(openingMove);
        /*
        for(int i = 1; i <= 3; i++){
            bot.followTrajectorySequenceAsync(cycles);
        }
         */

        return endPose; // returning our end position so our april tags conditionals know where to start from
    }
}