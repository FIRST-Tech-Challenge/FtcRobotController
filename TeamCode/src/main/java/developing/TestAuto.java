package developing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import autofunctions.Path;

@Autonomous(name="TestAuto", group="Auto")
public class TestAuto extends LinearOpMode {
    TestRobot bot = new TestRobot();
    Path2 path = new Path2(0,0,0);
    RobotFunctions2 rf = new RobotFunctions2();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

//        bot.startOdoThreadAuto(this);

        path.addStop(0.1);
        path.addRF(rf.intake(0.5));
        path.addStop(3);
        path.addRF(rf.intake(0), rf.startOuttake());
        path.addStop(5);
        path.addRF(rf.stopOuttake());
        path.start(bot, this);


//        bot.stopOdoThreadAuto();
    }

    public void initialize() {
        bot.init(hardwareMap);
        rf.init(bot);
    }
}
