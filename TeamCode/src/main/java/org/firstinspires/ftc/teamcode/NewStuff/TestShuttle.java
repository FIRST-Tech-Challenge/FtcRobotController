package org.firstinspires.ftc.teamcode.NewStuff;

import com.kuriosityrobotics.shuttle.HardwareTaskScope;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NewStuff.Localization.Odometry;
import org.firstinspires.ftc.teamcode.NewStuff.Localization.RobotMovement;
import org.firstinspires.ftc.teamcode.NewStuff.Math.Path;
import org.firstinspires.ftc.teamcode.NewStuff.Math.Point;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name = "joshua")
public class TestShuttle extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        try (var outer = HardwareTaskScope.open()){
            try (var scope = HardwareTaskScope.open()) {

                OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap ,this, telemetry);
                DriveTrain driveTrain = new DriveTrain(opModeUtilities);
                Odometry odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
                RobotMovement robotMovement = new RobotMovement(opModeUtilities, driveTrain, odometry);

                outer.fork(odometry::run);

                List<Point> pathPoints = new ArrayList<Point>() {{
                    add(new Point(0, 0));
                    add(new Point(600, 0));
                    add(new Point(600, 600));
                }};

                waitForStart();
                //code
                scope.fork(() -> robotMovement.pathFollow(new Path(pathPoints)));
                scope.fork(() -> driveTrain.setTestPower(0.5));
                telemetry.addLine("before scope.join");
                telemetry.update();
                scope.join();
                telemetry.addLine("after scope.join");
                telemetry.update();
            } finally {
                outer.shutdown();
                outer.join();
            }
        }
    }
}
