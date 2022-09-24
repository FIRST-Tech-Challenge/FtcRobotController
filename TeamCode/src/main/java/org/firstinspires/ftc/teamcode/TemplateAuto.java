package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.FourWheelDriveBot;

@Autonomous(name="TemplateAuto", group="Template")
@Disabled
public class TemplateAuto extends LinearOpMode {

    protected FourWheelDriveBot robot = new FourWheelDriveBot(this); //replace FourWheelDriveBot with whichever Bot is required

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        //call methods here
        robot.sleep(1000);
    }
}
