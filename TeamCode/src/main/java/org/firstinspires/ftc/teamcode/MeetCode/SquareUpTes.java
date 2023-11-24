package org.firstinspires.ftc.teamcode.MeetCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Square Up")

public class SquareUpTes extends LinearOpMode {
    Hardware robot = new Hardware();
    public void runOpMode(){
        robot.init(hardwareMap);

        waitForStart();
        while (robot.timer.seconds() < 30) {
            //robot.squareUp();
            telemetry.addData("Left", robot.distanceLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right", robot.distanceRight.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }


    }
}
