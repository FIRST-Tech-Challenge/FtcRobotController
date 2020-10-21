package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "firstTeleop", group = "awrlGelafv")
public class FirstAuto extends LinearOpMode {

    public DMHardware robot = new DMHardware(false);

    @Override
    public void runOpMode() {
        robot.initTeleOpIMU(hardwareMap);
        robot.timer.reset();

        robot.setPowerOfAllMotorsTo(0.3);

        while (robot.getTime() <= 5) {

        }

        robot.setPowerOfAllMotorsTo(0);
        robot.timer.reset();

        while (robot.getTime() <= 1) {

        }

        robot.timer.reset();

        robot.setPowerOfAllMotorsTo(0.3);
        while (robot.getTime() <= 5) {

        }
        robot.setPowerOfAllMotorsTo(0);


    }


}
