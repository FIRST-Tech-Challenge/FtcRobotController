package auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import autofunctions.Path;
import autofunctions.RobotFunctions;
import autofunctions.TerraCV;
import autofunctions.TerraCVHandler;
import global.TerraBot;
import globalfunctions.Constants;

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

        path.addWaypoint(0,10,0);
//        path.addSetpoint(0,40,0);
        path.addRF(rf.shootRF(3, op));
        path.addShoot(0,40, bot);
//
//
//
//        // Knock down tower, intake 1 ring, and outtake 1 ring
        path.addRF(rf.intake(1));
        path.addWaypoint(13,32,0);
        path.addStop(1.3);
        path.addRF(rf.shootRF(1, op));
        path.addShoot(0,0, bot);
//
//        // Intake 3 more rings, outtake them
        path.addRF(rf.intake(1));
        for (int i = 0; i < 3; i++) {
            path.addWaypoint(0, 20, 0);
            path.addStop(1);
        }
        path.addStop(0.5);
        path.addWaypoint(0,-10,0);
        path.addRF(rf.shootRF(3, op));
        path.addShoot(0,0, bot);
        //Place first wobble goal
        path.addWGRF(rf.controlWGE(1), rf.moveWgTo(0));
        path.addWaypoint(-55, 115, 0);
        path.addWGRF(rf.claw(2, -0.2));
        path.addWaypoint(0,30,0);
        //Go to 2nd wobble goal
        path.addWaypoint(10, -20,0);
        path.addWGRF(rf.moveWgTo(180));
        path.addWaypoint(70, -120,0);
        path.addWaypoint(0,-40,0);
        //Grab it and move to place
        path.addWGRF(rf.claw(0));
        path.addStop(1);
        path.addWGRF(rf.moveWgTo(130));
        path.addWaypoint(0,20,0);
        path.addWGRF(rf.controlWGE(1), rf.moveWgTo(0));
        path.addWaypoint(-80, 115, 0);
        path.addWGRF(rf.claw(2, -0.2));
        //Drop 2nd wobble goal
        path.addWaypoint(0,30,0);
        path.addStop(0.5);
        //Park
        path.addWaypoint(40,-60,0);
        path.addSetpoint(10,-20,0);




        path.start(bot, op);
//        path.saveData();

        bot.stopOdoThread();
        path.stopRFThread();

    }

    public void auto1(){

        // Shoot the first 3 rings
        path.addWGRF(rf.moveWgTo(55), rf.controlWGE(0.5));

        path.addWaypoint(0,10,0);
        path.addRF(rf.shootRF(3, op));
        path.addShoot(0,40, bot);


//
//        // Knock down tower, intake 1 ring, and outtake 1 ring
        path.addRF(rf.intake(1));
        path.addWaypoint(10,32,0);
        path.addStop(2);
        path.addRF(rf.shootRF(1, op));
        path.addShoot(0,0, bot);

        //Place first wobble goal
        path.addWGRF(rf.controlWGE(1), rf.moveWgTo(0));
        path.addWaypoint(-5, 95, 0);
        path.addWGRF(rf.claw(2, -0.2));
        path.addWaypoint(0,30,0);
        path.addStop(1);

        //Go to 2nd wobble goal
        path.addWaypoint(0, -20,0);
        path.addWGRF(rf.moveWgTo(190));
        path.addSetpoint(35, -10,0);
        path.addWaypoint(0, -80,0);
        //Grab it and move to place
        path.addWGRF(rf.claw(0));
        path.addStop(2);
        path.addWGRF(rf.moveWgTo(130));
        path.addWaypoint(0,20,0);

        path.addWGRF(rf.controlWGE(1), rf.moveWgTo(0));
        path.addWaypoint(-40, 40, 0);
        path.addWGRF(rf.claw(2, -0.2));
        //Drop 2nd wobble goal
        path.addWaypoint(0,30,0);
        path.addStop(0.5);
        //Park
        path.addSetpoint(0,-20,0);
//        path.addSetpoint(10,-20,0);




        path.start(bot, op);
//        path.saveData();

        bot.stopOdoThread();
        path.stopRFThread();
    }

    public void auto0(){

    }

    public void autoT(){
        path.addWaypoint(0, 30, 0);
        path.addSetpoint(0, 50, 0);
//        path.addStop(1);
//        path.addRF(rf.shootRF(3));
//        path.addShoot();
//
        path.start(bot, op);
//        path.saveData();
//
        bot.stopOdoThread();
        path.stopRFThread();
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
        bot.startOdoThreadAuto(op, true);
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
