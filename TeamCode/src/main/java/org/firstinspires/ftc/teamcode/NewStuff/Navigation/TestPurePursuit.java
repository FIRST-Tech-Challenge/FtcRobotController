package org.firstinspires.ftc.teamcode.NewStuff.Navigation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NewStuff.DriveTrain;
import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;
@TeleOp
public class TestPurePursuit extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap ,this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        Odometry odometry = new Odometry(driveTrain, this, telemetry, 0, 0, 0);
        PidNav pidNav = new PidNav(0, 0, 0);
        RobotMovement robotMovement = new RobotMovement(telemetry, driveTrain, odometry, pidNav);


        waitForStart();
        while (opModeIsActive()) {

            robotMovement.goToPosition(-300, 0, 135, 0);
        }
    }
}
