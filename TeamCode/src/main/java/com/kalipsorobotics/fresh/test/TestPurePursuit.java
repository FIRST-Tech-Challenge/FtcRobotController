package com.kalipsorobotics.fresh.test;

import com.kalipsorobotics.fresh.localization.Odometry;
import com.kalipsorobotics.fresh.localization.RobotMovement;
import com.kalipsorobotics.fresh.math.Point;
import com.kuriosityrobotics.shuttle.HardwareTaskScope;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.kalipsorobotics.fresh.math.Path;

import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;
import org.firstinspires.ftc.teamcode.NewStuff.modules.DriveTrain;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Fresh Pure Pursuit")
public class TestPurePursuit extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        try (var outer = HardwareTaskScope.open()) {
            // the outer scope is for the persistent tasks like odo
            // stuff, idk
            org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap ,this, telemetry);
            org.firstinspires.ftc.teamcode.NewStuff.modules.DriveTrain driveTrain = new DriveTrain(opModeUtilities);
            Odometry odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
            RobotMovement robotMovement = new RobotMovement(opModeUtilities, driveTrain, odometry);

            outer.fork(odometry::run);

            try (var inner = HardwareTaskScope.open()) {
                List<Point> pathPoints = new ArrayList<Point>() {{
                    add(new Point(0, 0));
                    add(new Point(600, 0));
                    add(new Point(600, 600));
                    add(new Point(1800, 600));
                    add(new Point(1800, 0));
                }};

                waitForStart();
                //code
                inner.fork(() -> robotMovement.pathFollow(new Path(pathPoints)));
                while (opModeUtilities.getOpMode().opModeIsActive()) {
                    idle();
                }
                inner.shutdown();
                inner.join();

                telemetry.update();
            } finally {
                outer.shutdown();
                outer.join();
            }
        }
    }
}
