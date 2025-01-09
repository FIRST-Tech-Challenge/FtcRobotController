package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Test extends Movable {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                moveClaws("grab");
            } else if (gamepad1.b) {
                moveClaws("release");
            }
        }
    }

    @Override
    void updatePhoneConsole() {

    }
}
