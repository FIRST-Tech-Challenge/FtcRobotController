package auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import autofunctions.Path;
import autofunctions.RobotFunctions;
import autofunctions.TerraCV;
import global.TerraBot;
import globalfunctions.Constants;


@Autonomous(name="AutoOne", group="Auto")
public class AutoOne extends LinearOpMode {
    AutoHandler autoHandler = new AutoHandler(this);
    @Override
    public void runOpMode() {
        autoHandler.initialize(false);
        autoHandler.auto1();
    }
}
