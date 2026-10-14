import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@TeleOp(name = "Logged TeleOp")
@PsiKitAutoLog
public class testOp extends OpMode {
    @Override
    public void init() {
        Logger.recordMetadata("Robot", "MyRobot");
        Logger.recordMetadata("Mode", "TeleOp");
    }

    @Override
    public void loop() {
        Logger.recordOutput("Loop/RuntimeSec", getRuntime());
        Logger.recordOutput("Drive/Cmd/Forward", -gamepad1.left_stick_y);
        Logger.recordOutput("Drive/Cmd/Turn", gamepad1.right_stick_x);
    }
}