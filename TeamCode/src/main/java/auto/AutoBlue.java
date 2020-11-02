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
        rf.telemetryText("done initing");
        //rf.generateRandomIM();
        waitForStart();
        bot.startOdoThreadAuto(this);


        path.addWaypoint(10,10,0);
        path.addWaypoint(-10,10,0);
        path.addSetpoint(-10,-10,0);
        path.addSetpoint(10,-10,0);
        path.start(bot, this);

        bot.move(0,0,0);
        bot.stopOdoThreadAuto();

    }
    private void initialize(){
        bot.init(hardwareMap);
        rf.init(bot, this);
    }
}
