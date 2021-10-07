package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutonDist extends LinearOpMode {
    Hardware robot    = new Hardware();
    double distIN = robot.dist.getDistance(DistanceUnit.INCH);

    @Override
    public void runOpMode() {

        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            // If the distance in centimeters is less than 10, set the power to 0.3
            if ( distIN < 10) {
                robot.flDrive.set(0.3);
                robot.frDrive.set(0.3);
                robot.blDrive.set(0.3);
                robot.brDrive.set(0.3);
            } else {  // Otherwise, stop the motors
                robot.flDrive.set(0);
                robot.frDrive.set(0);
                robot.blDrive.set(0);
                robot.brDrive.set(0);
            }
        }
    }
}
