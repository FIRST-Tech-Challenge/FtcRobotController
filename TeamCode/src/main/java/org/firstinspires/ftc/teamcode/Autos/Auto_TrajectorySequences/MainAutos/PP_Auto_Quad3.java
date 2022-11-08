
package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class PP_Auto_Quad3 extends PowerPlay_AprilTagDetection
{
    private boolean isAuto = true;
    //Declaring lift motor powers
    private Arm armControl;
    private Slide slideControl;
    private Claw clawMovement;

    // Declaring our motor PID for the lift; passing through our PID values
    public PP_Auto_Quad3()
    {
        super.runOpMode(); // runs the opMode of the apriltags pipeline

        waitForStart();

        armControl = new Arm(hardwareMap);
        slideControl = new Slide(hardwareMap);
        clawMovement = new Claw(hardwareMap, isAuto);
    }

    @Override
    public void runOpMode()
    {
        // bot object created
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);

        // calling our method to run trajectories and passing through our newly created bot object
        bot = runTrajectories(bot);

        // LOOPS TO RUN ASYNC \\
       while (opModeIsActive()){
            bot.update();
            slideControl.Update();
            armControl.update();
        }

    }// end of runOpMode()

    public SampleMecanumDrive runTrajectories(SampleMecanumDrive bot){
        Pose2d startPose = new Pose2d(-35, 61.8, Math.toRadians(270)); // x, y, heading (angle in radians)
        bot.setPoseEstimate(startPose);

        // TRAJECTORY SEQUENCES \\
        Pose2d endPose = cycle(bot, startPose); // cycle method -> should return a Pose2d

        // the tagUse variable is inherited from the AprilTagDetection class
        if(tagUse == 1) {
            TrajectorySequence sussyBaka = bot.trajectorySequenceBuilder(endPose)
                    .lineToLinearHeading(new Pose2d(-12.2, 15.5, Math.toRadians(-109)))
                    .build();
            bot.followTrajectorySequenceAsync(sussyBaka);
            telemetry.addData("Chris Pratt","Is Currently In The Mushroom Kingdom");
        }
        else if(tagUse ==2) {
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
                slideControl.setHighJunction();
                //slideControl.Update();
                armControl.setForwards();
                //armControl.update();
                clawMovement.toggleOpenClose();
                armControl.setBackwards();

                })//temporal marker for the extake
            .lineToLinearHeading(new Pose2d(-33,8,Math.toRadians(-39.75))) // Drive to cone stack

            .addTemporalMarker(1,()->{
                slideControl.setIntakeOrGround();
                //slideControl.Update();
                clawMovement.toggleOpenClose();
                armControl.setForwards();
                //armControl.update();
                })//marker for the intake, the timing will be tested so where the markers are located and times are subject to change

            .lineToLinearHeading(new Pose2d(-57, 12.3, Math.toRadians(0)))

            //.addTemporalMarker()
            .lineToLinearHeading(new Pose2d(-33, 8, Math.toRadians(-39.75)))
            .waitSeconds(1)//Deposit at junction; Under the impression that using the async PID, the slides will be already be moved up
            .build();

        Pose2d endPose = new Pose2d(-33,8,Math.toRadians(-39.75)); // Deposit at junction

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

        bot.followTrajectorySequenceAsync(openingMove);
        for(int i = 1; i <= 3; i++){
          bot.followTrajectorySequenceAsync(cycles);
         }
        return endPose; // returning our end position so our april tags conditionals know where to start from
    }

}
