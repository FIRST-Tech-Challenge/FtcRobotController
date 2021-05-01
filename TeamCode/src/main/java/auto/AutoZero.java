package auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name="AutoZero", group="Auto")
public class AutoZero extends LinearOpMode {
    AutoHandler autoHandler = new AutoHandler(this);
    @Override
    public void runOpMode() {
        autoHandler.initialize(false);
        autoHandler.auto0();
    }
}
