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
         *      PlAN
         *          1. shoot 3 rings into normal goal
         *          2. shoot 1 ring into normal goal
         *          3. shoot 3 rings into powershot
         *          4. place wobble goal
         *          5. get other wobble goal
         *          6. place other wobble goal
         *          7. park
         */
//        path.addRF(rf.intake(1));
        path.addRF(rf.resetAll(), rf.startOuttake());
        path.addSetpoint(0,20,0);
        path.addShoot();


        path.start(bot, this);

        bot.stopOdoThread();
    }

    public void initialize() {
        bot.init(hardwareMap);
        rf.init(bot);
        telemetry.addData("Ready:", ringNum);
        telemetry.update();
    }
}
