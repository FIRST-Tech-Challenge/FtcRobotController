package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Pinpoint Offsets")
public class PinpointOffsets extends LinearOpMode{
    GerthaHardwareMap hw;


    @Override
    public void runOpMode(){
        hw = new GerthaHardwareMap(hardwareMap);
        hw.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,0));
        waitForStart();
        while (opModeIsActive()) {
            hw.pinpoint.update();
            Pose2D currentPose = hw.pinpoint.getPosition();
            double currentX = currentPose.getX(DistanceUnit.INCH);
            double currentY = currentPose.getY(DistanceUnit.INCH);
            double currentTheta = currentPose.getHeading(AngleUnit.DEGREES);

            telemetry.addData("x", currentX);
            telemetry.addData("y", currentY);
            telemetry.addData("theta", currentTheta);
            telemetry.update();
        }

    }
}
