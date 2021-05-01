package auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name="AutoReal", group="Auto")
public class AutoReal extends LinearOpMode {
    AutoHandler autoHandler = new AutoHandler(this);

    @Override
    public void runOpMode() {
       autoHandler.initialize(true);
       autoHandler.autoAll();
    }

}
