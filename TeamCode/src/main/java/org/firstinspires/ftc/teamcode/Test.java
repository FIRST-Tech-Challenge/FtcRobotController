package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous
public class Test extends Movable {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        // ~3500 milliseconds to go to high rung, +200 to put it on

        // moves to high rung
        outtake.setPosition(0);
        Thread.sleep(250);

    }

    @Override
    void updatePhoneConsole() {

    }
}
