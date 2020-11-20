package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.movement.Mecanum;
import org.firstinspires.ftc.teamcode.pathFollow.Follower;
import org.firstinspires.ftc.teamcode.vuforia.VuMarkNav;


@TeleOp(name="Vuforia Seek", group="Linear Opmode")

public class VuforiaSeek extends LinearOpMode
{
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    String pathString = "src/main/java/org/firstinspires/ftc/teamcode/pathFollow/PathTXTs/testPath.txt";
    VuMarkNav vumark;
    Mecanum drivetrain;
    Follower pathFollower;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        vumark = new VuMarkNav(hardwareMap, telemetry);
        drivetrain = new Mecanum(hardwareMap);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


        waitForStart();

        vumark.activate();
        runtime.reset();
        pathFollower = new Follower(drivetrain, vumark, pathString, telemetry);

        while (opModeIsActive()){
            telemetry.addData("Destination: ", pathFollower.targetPose);
            telemetry.addData("Current Position: ", vumark.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
        }

        vumark.deactivate();
    }


}



