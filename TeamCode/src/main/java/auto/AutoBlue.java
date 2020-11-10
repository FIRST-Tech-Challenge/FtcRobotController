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
//        while (!isStarted() && !bot.isDoneResettingArm()) {
//            bot.resetArm();
//        }
        rf.telemetryText("done initializing");
//        rf.scanRings();
        waitForStart();


        bot.startOdoThreadAuto(this);
        path.addWaypoint(-25,50,0);
        path.addWaypoint(0, 40, 0);
        path.addRF(rf.shoot());
        path.addWaypoint(35,48,0);
        path.addSetpoint(0,10,0);
        path.addRF(rf.shoot());
        path.addStop(5);
        path.start(bot, this);
        bot.stopOdoThreadAuto();

    }
    private void initialize(){
       // bot.grabStart = 0.45;
        bot.init(hardwareMap);
        rf.init(bot, this);
    }
}
