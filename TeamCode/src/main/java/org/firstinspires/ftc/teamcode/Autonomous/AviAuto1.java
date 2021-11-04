package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.JacksonDriveTrain;

@Autonomous(name = "AviAuto1", group = "Autos")
public class AviAuto1 extends LinearOpMode{
    JacksonDriveTrain dt;
    @Override
    public void runOpMode() {
        waitForStart();
        dt = new JacksonDriveTrain(this);
        dt.spinCarousel();
        dt.driveAtHeading(125, 0, 31.8, 0.7);
        dt.driveAtHeading(125, 27.4, 0, 0.7);
        // color sensor range of 5cm
        while(!dt.isTS() && opModeIsActive()) {
             dt.driveAtHeading(125, 0, 24, 0.6);
        }
    }
}
