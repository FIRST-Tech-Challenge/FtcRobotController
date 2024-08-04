package org.firstinspires.ftc.teamcode.NewStuff.Localization;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NewStuff.DriveTrain;
import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;


@TeleOp
public class TestOdometry extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap ,this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        Odometry odometry = new Odometry(driveTrain, this, telemetry, 0, 0, Math.toRadians(0));
        RobotMovement robotMovement = new RobotMovement(opModeUtilities, driveTrain, odometry);

        waitForStart();
        while (opModeUtilities.getOpMode().opModeIsActive()) {
            odometry.updatePosition();
        }
    }
}
