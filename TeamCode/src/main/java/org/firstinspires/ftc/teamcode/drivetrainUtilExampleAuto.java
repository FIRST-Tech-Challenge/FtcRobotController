package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "drivetrainUtilExampleAuto", group = "Drive")
public class drivetrainUtilExampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Initialize drivetrain
        Drivetrain drivetrain = new Drivetrain(hardwareMap, 2, 5, 20);

        waitForStart();
        while (opModeIsActive()) {
            // stick to the back wall at a 20cm distance
            drivetrain.alignToWall(Drivetrain.WallType.BACK, 20, 0);
            drivetrain.update();
        }
        drivetrain.stop(); // stop drivetrain motors when done
    }
}