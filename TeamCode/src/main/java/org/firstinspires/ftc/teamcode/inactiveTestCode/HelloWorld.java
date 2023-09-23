package org.firstinspires.ftc.teamcode.inactiveTestCode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "Hello World", group="Robot")
@Disabled
public class HelloWorld extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Hello World! Add me in Genshin pls my UID is 642041765");
        telemetry.update();

        waitForStart();
        int count = 15000000;
        while (opModeIsActive()) {
            count--;
            telemetry.addLine("Genshin UID: 642041765");
            telemetry.addLine("Time until Arlecchino banner (maybe idk i guessed):");
            telemetry.addData("Years (generally)", count/60/60/24/7/4/12);
            telemetry.addData("Months (generally)", count/60/60/24/7/4);
            telemetry.addData("Weeks", count/60/60/24/7);
            telemetry.addData("Days", count/60/60/24);
            telemetry.addData("Hours", count/60/60);
            telemetry.addData("Minutes", count/60);
            telemetry.addData("Seconds", count);
            telemetry.addLine("(; - ;) <-- me bc Arlecchino only got 0.2 seconds of screentime in the 4.1 trailer");
            telemetry.update();
            sleep(100);
        }
    }
}
