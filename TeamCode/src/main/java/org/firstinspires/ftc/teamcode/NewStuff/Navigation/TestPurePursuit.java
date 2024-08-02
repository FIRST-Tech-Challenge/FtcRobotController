package org.firstinspires.ftc.teamcode.NewStuff.Navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NewStuff.DriveTrain;
import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;

import java.util.ArrayList;

@TeleOp
public class TestPurePursuit extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap ,this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        Odometry odometry = new Odometry(driveTrain, this, telemetry, 0, 0, 0);
        PidNav pidNav = new PidNav(0, 0, 0);
        RobotMovement robotMovement = new RobotMovement(opModeUtilities, driveTrain, odometry, pidNav);


        ArrayList<Point> path = new ArrayList<Point>() {{
            add(new Point(0, 0));
            add(new Point(0, 600));
            add(new Point(600, 600));
        }};

        waitForStart();
        while (opModeIsActive()) {
            robotMovement.targetPosition(0, 600, 0, 0);
            //robotMovement.pathFollow(path);
        }
    }
}
