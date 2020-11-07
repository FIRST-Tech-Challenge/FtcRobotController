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

@Autonomous(name = "AutoBlue", group = "Auto")
public class AutoBlue extends LinearOpMode {
    TerraBot bot = new TerraBot();
    RobotFunctions rf = new RobotFunctions();
    Path path = new Path(0,0,0);

    @Override
    public void runOpMode() {
        initialize();
        //rf.generateRandomIM();
        while (!isStarted() && !bot.isDoneResettingArm()) {
            bot.resetArm();
        }
        rf.telemetryText("done initing");
        waitForStart();
        bot.startOdoThreadAuto(this);
        path.addWaypoint(-25,50,45);
        path.addRF(rf.wobbleArm(150,1));
        path.addWaypoint(0, 100, 135);
        path.addRF(rf.turnArm(0.68));
        path.addSetpoint(0,40,0);
        path.addRF(rf.grab(0));
        path.start(bot, this);

        bot.move(0,0,0);
        bot.stopOdoThreadAuto();

    }
    private void initialize(){
        bot.grabStart = 0.45;
        bot.init(hardwareMap);
        rf.init(bot, this);
    }
}
