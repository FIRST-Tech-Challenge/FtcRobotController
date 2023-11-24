package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveMethods;

@TeleOp(name = "BasicSlideRotation", group = "LinearSlide")
public class BasicSlideRotation extends DriveMethods {
    @Override
    public void runOpMode() {
        initMotorsSecondBot();

        waitForStart();

        while(opModeIsActive()) {
            
        }
    }
}
