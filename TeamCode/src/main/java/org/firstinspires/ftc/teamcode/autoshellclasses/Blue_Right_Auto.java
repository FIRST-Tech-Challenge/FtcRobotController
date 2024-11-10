package org.firstinspires.ftc.teamcode.autoshellclasses;
import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.bluebananas.ftc.roadrunneractions.ActionBuilder;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.vision.VisionPortal;


// JOSHUANOTE: Change Auto name here.
@Config
@Autonomous(name = "Blue_Right_Auto", group = "Autonomous")
public class Blue_Right_Auto extends LinearOpMode {
    // JOSHUANOTE: initialization of servos and hardware mapping is here.
//-------------------------------------------------------------------------------------------


    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(-15, 63, Math.toRadians(-90));
        // JOSHUANOTE: Here is where the trajectories are intitialized and defined.
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));


        Action PushSample = ActionBuilder.BlueRightOption1(drive::actionBuilder);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }


        telemetry.update();
        waitForStart();


        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        // JOSHUANOTE: This is where you put the final set of actions.
                        //ActionBuilder.BlueRightOption1(drive::actionBuilder)
                        PushSample
                )
        );

        telemetry.update();
    }
}
