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

    RobotFunctions rf = new RobotFunctions();

    @Override
    public void runOpMode() {
        initialize();
        while (!isStarted() && !bot.isDoneResettingArm()) {
            bot.resetArm();
        }
        telemetry.addData("ready", "yes");
        telemetry.update();
        //rf.telemetryText("done initializing");
        waitForStart();
        bot.startOdoThreadAuto(this);
        path.addStop(1);
        path.addRF(rf.turnArm(0.68), rf.wobbleArm(180,1));
        path.addStop(2);
//        path.addStop(1);
//        path.addRF(rf.changeAcc(3, 2, 3, path));
//        path.addSetpoint(0,30,0);
//        path.addRF(rf.intake(1));
//        path.addStop(0.3);
//        path.addSetpoint(0, 3, 0);
//        path.addRF(rf.changeKs(0.4, path));
//        path.addWaypoint(0, 20, 0);
       // path.addWaypoint(0,5,0);
        //path.addStop(1);
        path.start(bot, this);
        bot.stopOdoThreadAuto();

    }
    private void initialize(){
        bot.init(hardwareMap);
        rf.init(bot, this);
    }

}
