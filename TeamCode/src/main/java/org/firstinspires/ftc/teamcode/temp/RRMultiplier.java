package org.firstinspires.ftc.teamcode.temp;

import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(group = "T")
public class RRMultiplier extends LinearOpMode {

    double randomMultiplier = 0.4;

    @Override
    public void runOpMode() {

        Rotation2d heading = Rotation2d.exp(2.1);

        telemetry.addLine("ORIGINAL: "+heading.toDouble());

        heading = heading.plus(randomMultiplier);

        telemetry.addLine("PLUS: "+heading.toDouble());


        telemetry.addLine("--------------------");

        Rotation2d oldHeading = Rotation2d.exp(2.1);

        telemetry.addLine("OLD METHOD INPUT"+oldHeading.toDouble());

        double correctedYaw = 2.1+randomMultiplier;

        oldHeading = Rotation2d.exp(correctedYaw);

        telemetry.addLine("OLD METHOD OUTPUT"+oldHeading.toDouble());


        telemetry.update();
        waitForStart();


    }

}
