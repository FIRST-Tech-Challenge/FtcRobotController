package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Test extends Movable {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        if (gamepad1.a) {
            FRScissorLift.setPower(1);
        } else if (gamepad1.b) {
            FLScissorLift.setPower(1);
        } else if (gamepad1.x) {
            BRScissorLift.setPower(-1);
        } else if (gamepad1.y) {
           BLScissorLift.setPower(-1);
        }
    }

    @Override
    void updatePhoneConsole() {

    }
}
