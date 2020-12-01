package auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import autofunctions.Path;
import autofunctions.RobotFunctions;
import autofunctions.TerraCV;
import global.TerraBot;
import util.CodeSeg;
import util.Rect;
import autofunctions.TerraCV.RingNum;


@Autonomous(name = "AutoTest", group = "Auto")
public class AutoTest extends LinearOpMode {
    TerraBot bot = new TerraBot();
    //RobotFunctions rf = new RobotFunctions();
    Path path = new Path(0,0,0);

    @Override
    public void runOpMode() {
        initialize();
        telemetry.addData("ready", "yes");
        telemetry.update();
        //rf.telemetryText("done initializing");
        waitForStart();
        bot.startOdoThreadAuto(this);
        path.addWaypoint(30,30,90);
        path.start(bot, this);
        bot.stopOdoThreadAuto();

    }
    private void initialize(){
        bot.init(hardwareMap);
        //rf.init(bot, this);
    }

}
