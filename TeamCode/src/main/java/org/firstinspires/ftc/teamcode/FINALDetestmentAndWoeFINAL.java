package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FINALDetestmentAndWoeFINAL extends Movable {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        // ~3500 milliseconds to go to high rung, +200 to put it on
        // after constricting outtake, must use Thread.sleep(500)
        // moves to high rung
        outtake.setPosition(0);
        powerWheels(200, "left");
        powerWheels(1500, "forward");
        powerScissorLift(3700, "erect");

        // hang specimen
        powerWheels(550, "forward");
        powerScissorLift(800, "descend");
        powerWheels(1300, "backward");

        // go to starting pos (2nd tile)
        powerScissorLift(2850, "descend");
        powerWheels(1000, "right");

        // first sample
        powerWheels(2450, "forward");
        powerWheels(400, "right");
        powerWheels(2700, "backward");

        // second sample
        powerWheels(2700, "forward");
        powerWheels(420, "right");
        powerWheels(2700, "backward");

        // wait for human player to attach
        powerWheels(750, "forward");
        turn180();

        //gets the first specimen
        powerScissorLift(500, "erect");
        outtakeGrab("liberation");
        powerWheels(1200, "forward");
        outtake.setPosition(0);
        Thread.sleep(500);
        powerScissorLift(300, "erect");
        powerWheels(750, "backward");
        turn180();
        powerScissorLift(300, "descend");

        // go hang new specimen
        powerWheels(1900, "left");
        powerScissorLift(3700, "erect");

        // hang specimen
        powerWheels(700, "forward");
        powerScissorLift(900, "descend");
        powerWheels(1300, "backward");
        // works! now has 2 specimen

        // goes back to pick new specimen
        powerScissorLift(2750, "descend");
        turn180();
        powerWheels(300, "backward");
        powerWheels(1900, "left");

        // goes to collect specimen
        powerScissorLift(500, "erect");
        outtakeGrab("liberation");
        powerWheels(1200, "forward");
        outtake.setPosition(0);
        Thread.sleep(500);
        powerScissorLift(300, "erect");
        powerWheels(750, "backward");
        turn180();
        powerScissorLift(300, "descend");

        // go hang new specimen
        powerWheels(2050, "left");
        powerScissorLift(3700, "erect");

        // hang specimen
        powerWheels(700, "forward");
        powerScissorLift(900, "descend");
        powerWheels(1300, "backward");
        powerScissorLift(2850, "descend");
        // works! now has 3 specimen

        // third sample (hits the wall though)
//        powerWheels(2700, "forward");
//        powerWheels(300, "right");
//        powerWheels(3000, "backward");

        telemetry.addData("Status", "Completed");
        telemetry.update();
    }

    @Override
    void updatePhoneConsole() {

    }
}
