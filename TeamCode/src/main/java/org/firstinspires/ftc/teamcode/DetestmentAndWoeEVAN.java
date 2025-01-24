package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class DetestmentAndWoeEVAN extends Movable{

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        // autonomous for scoring 4 samples, loading 1 on the left and scoring 2 more alliance-neutral ones
        // second one would be pushed forward but not all the way, needed to be successfully scored through tele-op
        // if needed for right side of the alliance, change all "left" arguments to "right"

        powerWheels(600, "left");
        powerWheels(600, "right");
        powerWheels(3200, "forward");
        turn180();
        powerWheels(425, "right");
        powerWheels(2750, "forward");
        powerWheels(2750, "backward");
        powerWheels(400, "right");
        powerWheels(2750, "forward");
        powerWheels(2750, "backward");
        powerWheels(300, "right");
        powerWheels(3000, "forward");

        disablePower();
        telemetry.addData("Status", "Completed");
        telemetry.update();

    }
        void updatePhoneConsole() {

    }
}
