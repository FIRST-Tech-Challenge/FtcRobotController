package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "drivetrainUtilExampleAuto", group = "Drive")
public class drivetrainUtilExampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Initialize drivetrain
        Drivetrain drivetrain = new Drivetrain(hardwareMap, 2, 3, 30, 20, 2000);

        waitForStart();
        while (opModeIsActive()) {
            // stick to the back wall at a 20cm distance
            drivetrain.alignToWall(Drivetrain.WallType.BACK, 40);
            drivetrain.alignToWall(Drivetrain.WallType.LEFT, 40);
            drivetrain.update();

            telemetry.addData("isAtTarget: ", drivetrain.isAtTarget);
            telemetry.addData("horz: ", drivetrain.horizontalDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("vert: ", drivetrain.verticalDistanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("xWeights: ", drivetrain.xWeights.toString());
            telemetry.addData("yWeights: ", drivetrain.yWeights.toString());
            telemetry.addData("rWeights: ", drivetrain.rWeights.toString());
            telemetry.update();

            if (drivetrain.isAtTarget) {
                break;
            }
        }
        drivetrain.stop();

        while (opModeIsActive()) {
            drivetrain.setAngle(-45);
            drivetrain.update();
        }
        drivetrain.stop(); // stop drivetrain motors when done
    }
}