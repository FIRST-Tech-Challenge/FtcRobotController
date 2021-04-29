package auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import autofunctions.Path;
import autofunctions.RobotFunctions;
import autofunctions.TerraCV;
import autofunctions.TerraCVHandler;
import global.TerraBot;
import globalfunctions.Constants;
import globalfunctions.Storage;

public class AutoHandler {
    TerraBot bot = new TerraBot();
    Path path = new Path(Constants.AUTO_START[0],Constants.AUTO_START[1],Constants.AUTO_START[2]);
    RobotFunctions rf = new RobotFunctions();
    TerraCV.RingNum ringNum;
    TerraCVHandler terraCVHandler = new TerraCVHandler();
    LinearOpMode op;


    public AutoHandler(LinearOpMode op){
        this.op = op;
    }

    public void auto4(){

        // Shoot the first 3 rings
        path.addWGRF(rf.moveWgTo(55), rf.controlWGE(0.5));
        path.addWaypoint(10, 20, 0);
        path.addRF(rf.shootRF(3));
        path.addShoot(0,40);


        // Knock down tower, intake 1 ring, and outtake 1 ring
        path.addRF(rf.intake(1));
        path.addWaypoint(0,22,0);
        path.addStop(1.3);
        path.addRF(rf.shootRF(1));
        path.addShoot(0,0);

        // Intake 3 more rings, outtake them
        path.addRF(rf.intake(1));
        for (int i = 0; i < 3; i++) {
            path.addWaypoint(0, 15, 0);
            path.addStop(1);
        }
        path.addStop(0.5);
        path.addRF(rf.shootRF(3));
        path.addShoot(0,-10);
        //Place first wobble goal
        path.addWGRF(rf.controlWGE(1), rf.moveWgTo(0));
        path.addWaypoint(-55, 110, 0);
        path.addWGRF(rf.claw(2, -0.2));
        path.addSetpoint(0,30,0);
        //Go to 2nd wobble goal
        path.addWaypoint(10, -20,0);
        path.addWGRF(rf.moveWgTo(180));
        path.addWaypoint(70, -120,0);
        path.addWaypoint(0,-30,0);
        //Grab it and move to place
        path.addWGRF(rf.claw(0));
        path.addStop(1);
        path.addWGRF(rf.moveWgTo(130));
        path.addWaypoint(0,20,0);
        path.addWGRF(rf.controlWGE(1), rf.moveWgTo(0));
        path.addWaypoint(-80, 115, 0);
        path.addWGRF(rf.claw(2, -0.2));
        //Drop 2nd wobble goal
        path.addSetpoint(0,30,0);
        path.addStop(0.5);
        //Park
        path.addWaypoint(40,-60,0);
        path.addSetpoint(10,-20,0);




        path.start(bot, op);
        path.saveData();

        bot.stopOdoThread();

//        //Life wobble arm and shoot rings
//        path.addWGRF(rf.moveWgTo(60), rf.controlWGE(0.3));
//        path.addRF(rf.readyShooter(), rf.shootIntoGoal(3), rf.stopOuttake());
//        path.addWaypoint(-30, 50, 0);
//        path.addShoot(0,60,0);
//
//        //drop 1st wobble goal
//        path.addWGRF(rf.controlWGE(1), rf.moveWgTo(12));
//        path.addWaypoint(0,100, 0);
//        path.addSetpoint(-10,50,0);
//        path.addWGRF(rf.claw(2, -0.2));
//        path.addStop(0.3);
//
//        //move to intake rings
//        path.addWGRF(rf.moveWgTo(60));
//        path.addRF(rf.controlWGE(0.1));
//        path.addWaypoint(0, -205, 0);
////        path.addSetpoint(0, -55, 0);
//        path.addSetpoint(35,10, 0);
//
//
//        //intake 3 rings
//        path.addRF(rf.intake(1));
//        path.addWGRF(rf.moveWgTo(180));
//        path.addStop(0.3);
//        path.addSetpoint(0,15,0);
//        path.addStop(0.3);
//        path.addSetpoint(0,15,0);
//        path.addStop(0.3);
//        path.addSetpoint(0,15,0);
//
//
//        //move and pick up 2nd wobble
//        path.addWaypoint(0,-20,0);
//        path.addSetpoint(5, -25, 20);
//        path.addWGRF(rf.claw(0), rf.pauseRfs(0.5), rf.moveWgTo(70));
//        path.addStop(1);
//
//        //shoot 3 rings
//        path.addWaypoint(-5, 15, -20);
//        path.addRF(rf.intake(0), rf.overrideShooter(true),  rf.readyShooter(), rf.pauseRfs(1.5),  rf.shootIntoGoal(3), rf.stopOuttake());
//        path.addStop(1);
//        path.addShoot(0, 35, 1);
//
//        //drop 2nd wobble goal
//        path.addWGRF(rf.controlWGE(1), rf.moveWgTo(12));
//        path.addRF(rf.intake(1));
//        path.addWaypoint(0,40,0);
//        path.addSetpoint(-50, 80, 0);
//        path.addWGRF(rf.claw(2, -0.2));
//        path.addStop(0.5);
//
//
//        //shoot last ring
////        path.addRF(rf.intake(0), rf.overrideShooter(true),  rf.readyShooter(), rf.pauseRfs(2.5),  rf.shootIntoGoal(1), rf.stopOuttake());
////        path.addWaypoint(30, -60, 0);
////        path.addShoot(20, -60, 0);
//
//        //park
//        path.addSetpoint(50, -90, 0);
//        path.addRF(rf.saveForTele());
//        path.addStop(1);
////
////        //park
////        path.addSetpoint(0,30,0);
//
//        path.start(bot, op);
//        path.saveData();
//
//
//        bot.stopOdoThread();
    }

    public void auto1(){

//        //Life wobble arm and shoot rings
//        path.addWGRF(rf.moveWgTo(60), rf.controlWGE(0.3));
//        path.addRF(rf.readyShooter(), rf.shootIntoGoal(3), rf.stopOuttake());
//        path.addWaypoint(-30, 50, 0);
//        path.addShoot(0,60,0);
//
//        //drop 1st wobble goal
//        path.addWGRF(rf.controlWGE(1), rf.moveWgTo(12));
//        path.addWaypoint(0,100, 0);
//        path.addSetpoint(-10,50,0);
//        path.addWGRF(rf.claw(2, -0.2));
//        path.addStop(0.3);
//
//        //move to intake rings
//        path.addWGRF(rf.moveWgTo(60));
//        path.addRF(rf.controlWGE(0.1));
//        path.addWaypoint(0, -150, 0);
//        path.addSetpoint(0, -55, 0);
//        path.addSetpoint(35,10, 0);
//
//
//        //intake 1 rings
//        path.addRF(rf.intake(1));
//        path.addWGRF(rf.moveWgTo(180));
//        path.addStop(0.3);
//        path.addSetpoint(0,15,0);
//        path.addWaypoint(0, 30, 0);
//
//
//        //move and pick up 2nd wobble
//        path.addWaypoint(0,-20,0);
//        path.addSetpoint(5, -25, 20);
//        path.addWGRF(rf.claw(0), rf.pauseRfs(0.5), rf.moveWgTo(70));
//        path.addStop(1);
//
//        //shoot 1 rings
//        path.addWaypoint(-5, 15, -20);
//        path.addRF(rf.intake(0), rf.overrideShooter(true),  rf.readyShooter(), rf.pauseRfs(1.5),  rf.shootIntoGoal(1), rf.stopOuttake());
//        path.addStop(1);
//        path.addShoot(0, 35, 1);
//
//        //drop 2nd wobble goal
//        path.addWGRF(rf.controlWGE(1), rf.moveWgTo(12));
//        path.addRF(rf.intake(1));
//        path.addWaypoint(0,40,0);
//        path.addSetpoint(-50, 90, 0);
//        path.addWGRF(rf.claw(2, -0.2));
//        path.addStop(0.5);
//
//
//        //park
//        path.addSetpoint(50, -90, 0);
//
//        path.addRF(rf.saveForTele());
//        path.addStop(1);
//
//        path.start(bot, op);
//        path.saveData();
//
//
//        bot.stopOdoThread();
    }

    public void auto0(){

//        //Life wobble arm and shoot rings
////        path.addWGRF(rf.moveWgTo(80), rf.controlWGE(0.3));
////        path.addRF(rf.readyShooter(), rf.shootIntoGoal(3), rf.stopOuttake());
//        path.addWaypoint(0, 50, 0);
////        path.addShoot(0,30,0);
//        path.addSetpoint(0,60,0);
//
//        //drop 1st wobble goal
////        path.addWGRF(rf.controlWGE(1), rf.moveWgTo(-10));
////        path.addWaypoint(-20,100, 0);
////        path.addSetpoint(-35,50,0);
////        path.addWGRF(rf.claw(2, -0.2));
////        path.addStop(1);
////
////        path.addWaypoint(25, -100, 0);
////        path.addSetpoint(50, -60, 175);
////        path.addSetpoint(0, -60, 0);
////        path.addWGRF(rf.claw(0), rf.pauseRfs(0.5), rf.moveWgTo(60));
////        path.addStop(1);
////        path.addWaypoint(0, 60, 0);
////        path.addSetpoint(-50, 60, -175);
////        path.addWGRF(rf.moveWgTo(12));
////        path.addSetpoint(-10, 100, 0);
////        path.addWGRF(rf.claw(2, -0.2));
////        path.addStop(1);
////
////        path.addSetpoint(50, -90, 0);
////
////        path.addRF(rf.saveForTele());
////        path.addStop(1);
//
//        path.start(bot, op);
//        path.saveData();
//
//        bot.stopOdoThread();
    }

    public void autoT(){
//        path.addWaypoint(0, 10, 0);
//        path.addRF(rf.shootRF(3));
//        path.addShoot();
//
//        path.start(bot, op);
//        path.saveData();
//
//        bot.stopOdoThread();
    }

    public void autoAll(){
        switch(ringNum){
            case ZERO:
                auto0();
            case ONE:
                auto1();
            case FOUR:
                auto4();
        }
    }




    public void initialize(boolean scan) {
        bot.angularPosition.dontUseCompassSensor = true;
        bot.init(op.hardwareMap);
        bot.wgStart = Constants.WG_START_POS_AUTON;
        rf.init(bot);
        bot.startOdoThreadAuto(op);
        path.startRFThread(op);
        if(scan) {
            //Uncomment this if u want to see the vid
            if(terraCVHandler.terraCV.show) {
                terraCVHandler.init(op.hardwareMap);
            }else{
                terraCVHandler.init();
            }
//           //Comment this out if u want to see vid
//            terraCVHandler.init();
            while (!op.isStarted()) {
                ringNum = terraCVHandler.getRingNum();
                if(terraCVHandler.terraCV.show) {
                    op.telemetry.addData("Ready:", ringNum);
                    op.telemetry.addData("Avgh", terraCVHandler.terraCV.avgH);
                    op.telemetry.addData("Avgs", terraCVHandler.terraCV.avgS);
                    op.telemetry.addData("Avgv", terraCVHandler.terraCV.avgV);
                    op.telemetry.update();
                }else{
                    op.telemetry.addData("Ready:", ringNum);
                    op.telemetry.update();
                }
            }
            terraCVHandler.stop();
        }else{
            op.telemetry.addData("Ready:", "Yes?");
            op.telemetry.update();
        }
        op.waitForStart();

    }
}
