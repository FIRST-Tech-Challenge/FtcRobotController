package auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import autofunctions.Path;
import autofunctions.RobotFunctions;
import developing.TerraCV;
import developing.TerraCVHandler;
import global.TerraBot;

@Disabled
@Autonomous(name="TestAuto", group="Auto")
public class TestAuto extends LinearOpMode {
    TerraBot bot = new TerraBot();
    Path path = new Path(0,0,0);
    RobotFunctions rf = new RobotFunctions();
    TerraCVHandler terraCVHandler = new TerraCVHandler();
    TerraCV.RingNum ringNum;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        terraCVHandler.stop();



//
//
//        bot.startOdoThreadAuto(this);
//
////        path.addStop(0.1);
////
////
////        path.addRF(rf.startOuttake(bot));
////        path.addStop(5);
////        path.addRF(rf.stopOuttake());
//
//        path.addSetpoint(20,20,90);
//        path.addSetpoint(10,10,10);
//
//        path.start(bot, this);
//
//
//        bot.stopOdoThread();
    }

    public void initialize() {
        bot.init(hardwareMap);
        rf.init(bot);
        terraCVHandler.init();
        while (!isStarted()) {
            ringNum = terraCVHandler.getRingNum();
            telemetry.addData("Ready:", ringNum);
            telemetry.update();
        }
    }
}
