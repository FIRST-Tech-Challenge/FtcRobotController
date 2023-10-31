package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoMethods.Direction.left;
import static org.firstinspires.ftc.teamcode.AutoMethods.Direction.right;
import static org.firstinspires.ftc.teamcode.AutoMethods.Direction.back;
import static org.firstinspires.ftc.teamcode.AutoMethods.Direction.forward;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.AutoMethods.*;

@Autonomous(name="AutoZamer", group="Auto")
//Disabled
public class AutoZamer extends LinearOpMode {

    AutoMethods bot = new AutoMethods();

    //ОСНОВНАЯ ПРОГРАММА
    @Override
    public void runOpMode() throws InterruptedException {

       bot.camStart(this);
        waitForStart();
        bot.start();
        bot.drive_passive(0, 1400, 0.65, this);
    }
}