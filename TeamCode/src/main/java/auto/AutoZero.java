package auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import autofunctions.Path;
import autofunctions.RobotFunctions;
import developing.TerraCV;
import developing.TerraCVHandler;
import global.TerraBot;
import globalfunctions.Constants;

@Autonomous(name="AutoZero", group="Auto")
public class AutoZero extends LinearOpMode {
    TerraBot bot = new TerraBot();
//    Path path = new Path(Constants.START_X,Constants.START_Y,Constants.START_H);
    Path path = new Path(Constants.START_X,Constants.START_Y,Constants.START_H);
    RobotFunctions rf = new RobotFunctions();
    TerraCV.RingNum ringNum = TerraCV.RingNum.ZERO;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        bot.startOdoThreadAuto(this);

        /**
         *
         *
         * TODO LIST
         *  Programming:
         *      auton
         *
         *      PlAN (TIME) [note]
         *          1. scan rings (0.0) [done before init]
         *          2. ready shooter (0.0) [done while raising wobble]
         *          3. raise wobble goal (1)
         *          4. move forward (1.5)
         *          5. shoot 3 rings into normal goal (3.5)
         *          6. turn intake on (3.5) [doesnt take time]
         *          7. move forward (4)
         *          8. intake 1 ring (5)
         *          9. ready shooter (6)
         *          10. shoot ring (7)
         *          11. intake on (7)
         *          12. intake 3 rings (10)
         *          13. ready shooter (11)
         *          14. move sideways (12)
         *          15. shoot powershots (15)
         *          16. move to wobble goal drop (18)
         *          17. move wobble arm (19)
         *          18. open claw (19.5)
         *          19. turn arm around (20.5)
         *          20. move to pick up other wobble (23.5)
         *          21. grab other wobble (24)
         *          22. pick up other wobble (25)
         *          23. place other wobble (28)
         *          24. open claw (28.5)
         *          24. move to park (30)
         *
         *
         */
        path.addRF(rf.readyShooter(), rf.shootIntoGoal(3), rf.stopOuttake());
        path.addShoot(7,50,0);

        path.start(bot, this);
        path.saveData();

        bot.stopOdoThread();
    }

    public void initialize() {
        bot.angularPosition.dontUseCompassSensor = true;
        bot.init(hardwareMap);
        rf.init(bot);
        telemetry.addData("Ready:", ringNum);
        telemetry.update();
    }
}
