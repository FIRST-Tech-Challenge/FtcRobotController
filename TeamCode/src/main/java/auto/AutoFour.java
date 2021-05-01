package auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Disabled
@Autonomous(name="AutoFour", group="Auto")
public class AutoFour extends LinearOpMode {
    AutoHandler autoHandler = new AutoHandler(this);
    @Override
    public void runOpMode() {
        autoHandler.initialize(false);
        autoHandler.auto4();
    }

}
