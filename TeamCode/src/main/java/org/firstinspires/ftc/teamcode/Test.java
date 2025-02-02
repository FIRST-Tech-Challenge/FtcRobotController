package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Test extends Movable {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while (opModeIsActive()) {


//            if (gamepad1.b) {
//                turnRotatingServos("rollUp");
//            } else if (gamepad1.a) {
//                turnRotatingServos("rollDown");
//            } else if (gamepad1.x) {
//                intakeGrab("constriction");
//            } else if (gamepad1.y) {
//                 intakeGrab("liberation");
//            }
        }

    }

    @Override
    void updatePhoneConsole() {

    }
}
