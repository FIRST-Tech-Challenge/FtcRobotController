package auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import autofunctions.Path;
import autofunctions.RobotFunctions;
import autofunctions.TerraCV;
import global.TerraBot;
import globalfunctions.Constants;

@Autonomous(name="AutoFour", group="Auto")
public class AutoFour extends LinearOpMode {
    TerraBot bot = new TerraBot();
    //    Path path = new Path(Constants.START_X,Constants.START_Y,Constants.START_H);
    Path path = new Path(Constants.AUTO_START[0],Constants.AUTO_START[1],Constants.AUTO_START[2]);
    RobotFunctions rf = new RobotFunctions();
    TerraCV.RingNum ringNum = TerraCV.RingNum.FOUR;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        /**
         *
         *
         *  PlAN NEW(TIME) [note]
         *          1. scan rings (0.0) [done before init]
         *          2. ready shooter (0.0) [done while raising wobble]
         *          3. raise wobble goal (1)
         *          4. move forward (3)
         *          5. shoot 3 rings into normal goal (6)
         *          6. ready wobble arm [done while moving] (0)
         *          7. move forward (7)
         *          8. place wobble goal (9)
         *          9. move back (12)
         *          10. pick wobble goal from back (13)
         *          11.
         */



        path.addWGRF(rf.moveWgTo(45), rf.controlWGE(0.3));
        path.addRF(rf.readyShooter(), rf.shootIntoGoal(3), rf.stopOuttake());
        path.addWaypoint(-30, 50, 0);
//        path.addWaypoint(0,30,0);
        path.addShoot(0,60,0);
        path.addWGRF(rf.controlWGE(1), rf.moveWgTo(0));
        path.addWaypoint(0,100, 0);
        path.addSetpoint(-10,50,0);
        path.addWGRF(rf.claw(2, -0.2));
        path.addStop(0.3);
        path.addWGRF(rf.moveWgTo(45));
        path.addRF(rf.controlWGE(0.1));
        path.addWaypoint(0, -225, 0);
        path.addSetpoint(35, 20, 0);
        path.addSetpoint(0,20, 0);
        path.addRF(rf.intake(1));
        path.addWGRF(rf.moveWgTo(150));
        path.addStop(1);
        path.addWaypoint(0,13,0);
        path.addStop(4);
        path.addRF(rf.intake(0), rf.readyShooter(), rf.pauseRfs(1),  rf.shootIntoGoal(2), rf.stopOuttake());
        path.addShoot(0, 15, 0);
//        path.addWaypoint(0,10,0);
//        path.addRF(rf.readyShooter(), rf.shootIntoGoal(2), rf.stopOuttake());
//        path.addStop(2);
//        path.addShoot();


        path.start(bot, this);
        path.saveData();

        bot.stopOdoThread();
    }

    public void initialize() {
        bot.angularPosition.dontUseCompassSensor = true;
        bot.init(hardwareMap);
        rf.init(bot);
        bot.startOdoThreadAuto(this);
        path.startRFThread(this);
        telemetry.addData("Ready:", ringNum);
        telemetry.update();

    }
//    @Override
//    public void stop(){
//
//    }
}
