package auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import autofunctions.Path;
import autofunctions.RobotFunctions;
import global.TerraBot;

@Autonomous(name="TestAuto", group="Auto")
public class TestAuto extends LinearOpMode {
    TerraBot bot = new TerraBot();
    Path path = new Path(0,0,0);
    RobotFunctions rf = new RobotFunctions();

    @Override
    public void runOpMode() {
        initialize();



        waitForStart();



        bot.startOdoThreadAuto(this);

//        path.addStop(0.1);
//
//
//        path.addRF(rf.startOuttake(bot));
//        path.addStop(5);
//        path.addRF(rf.stopOuttake());

        path.addSetpoint(20,20,90);
        path.addSetpoint(10,10,10);

        path.start(bot, this);


        bot.stopOdoThread();
    }

    public void initialize() {
        bot.init(hardwareMap);
        rf.init(bot);

        telemetry.addData("Ready", "");
        telemetry.update();
    }
}
