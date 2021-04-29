package telefunctions;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import autofunctions.Path;
import global.TerraBot;
import globalfunctions.Optimizer;
import globalfunctions.TerraThread;
import globalfunctions.Constants;
import util.CodeSeg;
import util.Stage;

public class AutoModule {

    //List of stages in the automodule
    public ArrayList<Stage> stages = new ArrayList<>();
    //Current stage that the automodule is on
    public int stageNum = 0;
    //Timer for internal methods
    public ElapsedTime timer = new ElapsedTime();

    //Is the automodule pausing?
    public boolean pausing =   false;
    //Path used for internal methods
    public Path path;
    //Terrabot for methods
    public TerraBot bot;

    //Code that updates in the loop
    public CodeSeg updateCode = new CodeSeg() {
        @Override
        public void run() {
            Stage s = stages.get(stageNum);
            if (s.run(timer.seconds())) {
                stageNum+=1;
                timer.reset();
            }
            if (stageNum == (stages.size())) {
                stageNum = 0;
            }
        }
    };

    //Thread for automodules to run in
    public TerraThread autoModuleThread;
    //Has the automodule been initalized yet?
    public boolean inited = false;
    //Initializes the automodule with a terrabot
    public void init(TerraBot bot){
        this.bot = bot;
    }
    //Starts the automodule and unpauses it
    public void start(){
        //If the thread hasnt been inited yet
        if(!inited) {
            //Define the automodule thread
            autoModuleThread = new TerraThread(updateCode);
            autoModuleThread.changeRefreshRate(Constants.AUTOMODULE_REFRESH_RATE);
            inited = true;
            Thread t = new Thread(autoModuleThread);
            t.start();
        }
        //Set pausing to false
        pausing = false;
    }
    //Is the automodule executing?
    public boolean isExecuting(){
        return autoModuleThread.executing;
    }
    //Stop the automodule
    public void stop(){
        autoModuleThread.stop();
    }

    //Add stage to control the wobble goal extender
    public void addControlWGE(final double pos){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.controlWGE(pos);
                bot.moveArm(bot.getRestPowArm());
                return bot.isControlWgeDone(pos);
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.wge.setPower(0);
                return true;
            }
        });
    }

    //Add stage to move the wobble goal arm to a certian pos
    public void addWobbleGoal(final double deg, final double pow){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.moveArmWithEncWithoutWGE(deg, pow);
                return true;
            }
        });
    }
    //Add stage to hold the wobble goal and pause the automodule
    public void holdWobbleGoalAndPause(){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                pausing = true;
                return true;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.moveArm(bot.getRestPowArm());
                return !pausing;
            }
        });
    }
    //Add Stage to close/open the claw
    public void addClaw(final int idx){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.claw(bot.cllControl.getPos(idx), bot.clrControl.getPos(idx));
                return true;
            }
        });
    }
    //Add stage to move CR servo at a certian pow
    public void addStage(final CRServo crs, final double pow) {
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                crs.setPower(pow);
                return true;
            }
        });
    }
    //Add stage to move dc motors at a certian pow
    public void addStage(final double pow, final DcMotor ...mots) {
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                for (DcMotor mot: mots) {
                    mot.setPower(pow);
                }
                return true;
            }
        });
    }

    //Add a pause
    public void addPause() {
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                pausing = true;
                return true;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                return !pausing;
            }
        });
    }
    //Waits for the amount of time specified
    public void addWait(final double t) {
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                return in > t;
            }
        });
    }
    //Adds a custom code block
    public void addCustom(final CodeSeg cs){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                cs.run();
                return true;
            }
        });
    }
    //Turs to the goal to shoot
    public void addTurnToGoal(){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.isMovementAvailable = false;
                bot.fastMode = false;
                path = new Path(bot.odometry.getAll());
                path.HAcc = 0.25;
                path.scaleControls(2);
                path.addSetpoint(0, 0, (bot.getRobotToGoalAngle()-bot.odometry.h));
                return true;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                double[] pows = path.update(bot.odometry.getAll(), bot);
                bot.move(pows[1], pows[0], pows[2]);
                return !path.isExecuting;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.isMovementAvailable = true;
                bot.move(0,0,0);
                return true;
            }
        });
    }
    //Move the robot accepting a pos and if the point is a waypoint
    public void addMove(final double[] move, final boolean way){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.isMovementAvailable = false;
                path = new Path(bot.odometry.getAll());
                path.HAcc = 0.25;
                path.XAcc = 0.5;
                path.YAcc = 0.5;
                if(way) {
                    path.addWaypoint(move[0], move[1], move[2]);
                }else{
                    path.addSetpoint(move[0], move[1], move[2]);
                }
                return true;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                double[] pows = path.update(bot.odometry.getAll(), bot);
                bot.move(pows[1], pows[0], pows[2]);
                return !path.isExecuting;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.isMovementAvailable = true;
                bot.move(0,0,0);
                return true;
            }
        });
    }
    //Changes autoaimer mode to certain mode
    public void changeAutoAimerMode(final int mode){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.autoAimer.shotMode = mode;
                return true;
            }
        });
    }

//    public void addMoveGlobal(final double[] pos){
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                bot.isMovementAvailable = false;
//                path = new Path(bot.odometry.getAll());
//                path.setGlobalMode(true);
//                path.HAcc = 0.25;
//                path.XAcc = 0.5;
//                path.YAcc = 0.5;
////                path.addWaypoint(pos[0], pos[1], Optimizer.optimizeHeading(pos[2]));
//                path.addSetpoint(pos[0], pos[1], Optimizer.optimizeHeading(pos[2]));
//                return true;
//            }
//        });
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                double[] pows = path.update(bot.odometry.getAll(), bot);
//                bot.move(pows[1], pows[0], pows[2]);
//                return !path.isExecuting;
//            }
//        });
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                bot.isMovementAvailable = true;
//                bot.move(0,0,0);
//                return true;
//            }
//        });
//    }
//    public void addStage(final DcMotor mot, final double pow, final int pos) {
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                mot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                mot.setTargetPosition(pos);
//                mot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                mot.setPower(pow);
//                return true;
//            }
//        });
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                return !mot.isBusy();
//            }
//        });
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                mot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                mot.setPower(0);
//                return true;
//            }
//        });
//
//    }
//
//    public void addSave(final Cycle c, final int idx){
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                c.curr = idx;
//                return true;
//            }
//        });
//    }
//    public void addSave(final ServoController c, final double pos){
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                c.cur = pos;
//                return true;
//            }
//        });
//    }


//
//
//    public void addStage(final DcMotor mot, final double pow, final double t) {
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                mot.setPower(pow);
//                return in > t;
//            }
//        });
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                mot.setPower(0);
//                return true;
//            }
//        });
//    }
//
//    public void addOuttake(final DcMotorEx outr, final DcMotorEx outl, final double outrVel, final double outlVel) {
//        if(outrVel == 0){
//            stages.add(new Stage() {
//                @Override
//                public boolean run(double in) {
//                    outr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    outl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    outr.setPower(0);
//                    outl.setPower(0);
//                    return true;
//                }
//            });
//        } else {
//            stages.add(new Stage() {
//                @Override
//                public boolean run(double in) {
//                    outr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    outl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    outr.setVelocity(outrVel);
//                    outl.setVelocity(outlVel);
//                    return true;
//                }
//            });
//        }
//    }
//    public void addWGE(final TerraBot bot){
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                if(bot.isWgeInLimits(1)){
//                    bot.updateWge();
//                }
//                return bot.isWgeDone();
//            }
//        });
//    }


//    public void addStage(final Servo s, final double pos, final double t) {
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                s.setPosition(pos);
//                return in > t;
//            }
//        });
//    }
//
//
//    public void addStage(final Servo s, final Cycle cycle, final int idx, final double t) {
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                cycle.curr = idx;
//                s.setPosition(cycle.getPos(idx));
//                return in > t;
//            }
//        });
//    }

//    public void addAimer(){
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) { bot.isMovementAvailable = false;
//               path = new Path(bot.odometry.getAll());
//               path.setGlobalMode(true);
//               bot.fastMode = false;
////               path.addSetpoint(Constants.TELE_START[0],Constants.TELE_START[1], Constants.TELE_START[2] );
//                path.addWaypoint(bot.aimerPos[0], bot.aimerPos[1], Optimizer.optimizeHeading(bot.aimerPos[2]));
//                path.addSetpoint(bot.aimerPos[0], bot.aimerPos[1], Optimizer.optimizeHeading(bot.aimerPos[2]));
//                return true;
//            }
//        });
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                double[] pows = path.update(bot.odometry.getAll(), bot);
//                bot.move(pows[1], pows[0], pows[2]);
//                return !path.isExecuting;
//            }
//        });
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                bot.isMovementAvailable = true;
//                bot.move(0,0,0);
//                return true;
//            }
//        });
//    }
//
//    public void addWaitForReached() {
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                return bot.autoAimer.hasReached;
//            }
//        });
//    }
//    public void addPath(final Path path, final TerraBot bot){
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                double[] pows = path.update(bot.odometry.getPos(), bot);
//                bot.move(pows[1], pows[0], pows[2]);
//                return !path.isExecuting;
//            }
//        });
//        stages.add(new Stage() {
//            @Override
//            public boolean run(double in) {
//                bot.move(0,0,0);
//                return true;
//            }
//        });
//    }


}
