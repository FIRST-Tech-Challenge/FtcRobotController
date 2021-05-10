package autofunctions;

import java.util.ArrayList;

import global.TerraBot;
import globalfunctions.Constants;
import util.CodeSeg;
import util.Stage;

public class RobotFunctions {
    //Terrabot to use for methods
    public TerraBot bot;
    //Path to use for methods
    public Path rfPath;
    //Initialize the bot
    public void init(TerraBot bot){
        this.bot = bot;
    }
    //Move the wobble goal extender
    public ArrayList<Stage> moveWGE(final double pos){
        ArrayList<Stage> stages = new ArrayList<>();
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
        return stages;
    }
    //Move the wobble goal arm
    public ArrayList<Stage> moveWGA(final double deg, final double pow){
        ArrayList<Stage> stages = new ArrayList<>();
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.moveArmWithEncWithoutWGE(deg, pow);
                return true;
            }
        });
        return stages;
    }
    //Move the wobble goal claw
    public ArrayList<Stage> moveClaw(final int idx, final double offset){
        ArrayList<Stage> stages = new ArrayList<>();
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.claw(bot.cllControl.getPos(idx)+offset, bot.clrControl.getPos(idx)-offset);
                return true;
            }
        });
        return stages;
    }
    //Turn towards the goal
    public ArrayList<Stage> turnToGoal(){
        ArrayList<Stage> stages = new ArrayList<>();
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.isMovementAvailable = false;
                bot.fastMode = false;
                rfPath = new Path(bot.odometry.getAll());
                rfPath.init(bot);
                rfPath.addSetpoint(0, 0, (bot.getRobotToGoalAngle()-bot.odometry.h));
                return true;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                double[] pows = rfPath.update(bot.odometry.getAll());
                bot.move(pows[1], pows[0], pows[2]);
                return !rfPath.isExecuting;
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
        return stages;
    }
    //Change autoaimer mode
    public ArrayList<Stage> changeAAMode(final int mode){
        ArrayList<Stage> stages = new ArrayList<>();
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.autoAimer.shotMode = mode;
                return true;
            }
        });
        return stages;
    }
    //Add a custom codeseg
    public ArrayList<Stage> addCustom(final CodeSeg cs){
        ArrayList<Stage> stages = new ArrayList<>();
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                cs.run();
                return true;
            }
        });
        return stages;
    }
    //Add a wait
    public ArrayList<Stage> addWait(final double time){
        ArrayList<Stage> stages = new ArrayList<>();
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                return in > time;
            }
        });
        return stages;
    }
    //Move the ring shooter
    public ArrayList<Stage> moveRS(final double pow){
        ArrayList<Stage> stages = new ArrayList<>();
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.rs.setPower(pow);
                return true;
            }
        });
        return stages;
    }
    //Outtake the rings
    public ArrayList<Stage> outtake(final double pow){
        ArrayList<Stage> stages = new ArrayList<>();
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.outr.setPower(pow);
                bot.outl.setPower(pow);
                return true;
            }
        });
        return stages;
    }
    //Intake the rings
    public ArrayList<Stage> intake(final double pow){
        ArrayList<Stage> stages = new ArrayList<>();
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.intake(pow);
                return true;
            }
        });
        return stages;
    }
    public ArrayList<Stage> shoot(final int numRings){
        ArrayList<Stage> stages = new ArrayList<>();
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.intake(0);
                bot.outtaking = true;
                bot.autoAimer.shotMode = 0;
                return true;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                return !bot.autoAimer.hasReached;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.rs.setPower(Constants.RS_POW);
                return true;
            }
        });
        stages.addAll(addWait(0.7));
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.outr.setPower(0);
                bot.outl.setPower(0);
                bot.rs.setPower(0);
                bot.outtaking = false;
                bot.autoAimer.done();
                return true;
            }
        });
        return stages;
    }



}
