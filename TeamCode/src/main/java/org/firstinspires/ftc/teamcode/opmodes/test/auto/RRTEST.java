
package org.firstinspires.ftc.teamcode.opmodes.test.auto;


import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;

/**
 * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
 */
@Autonomous(name = "RR Test")
@Disabled
public class RRTEST extends LinearOpMode {

    @Override
    public void runOpMode(){

        //Key Pad input to selecting Starting Position of robot
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        waitForStart();

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0)); // Starting Pose
        Pose2d submersibleSpecimen = new Pose2d(1,0,Math.toRadians(0) );
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

            //Move robot to submersible to place specimen
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(submersibleSpecimen.position, submersibleSpecimen.heading)
                            .build());
            safeWaitSeconds(1);
            telemetry.addLine("Move robot to submersible to place specimen on chamber");
            telemetry.update();

            // Add code for placing specimen on chamber and pick sample from inside submersible
            safeWaitSeconds(1);
            telemetry.addLine("Place specimen on chamber and pick sample from inside submersible");
            telemetry.update();
        }
    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
}

