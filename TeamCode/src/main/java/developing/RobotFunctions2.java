package developing;

import java.util.ArrayList;

import autofunctions.Path;
import global.TerraBot;
import util.CodeSeg;
import util.Stage;

public class RobotFunctions2 {
    public TerraBot bot;
    public Path rfPath;
    public void init(TerraBot bot){
        this.bot = bot;
    }
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

    public ArrayList<Stage> moveClaw(final int idx){
        ArrayList<Stage> stages = new ArrayList<>();
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.claw(bot.cllControl.getPos(idx), bot.clrControl.getPos(idx));
                return true;
            }
        });
        return stages;
    }
    public ArrayList<Stage> turnToGoal(){
        ArrayList<Stage> stages = new ArrayList<>();
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.isMovementAvailable = false;
                bot.fastMode = false;
                rfPath = new Path(bot.odometry.getAll());
                rfPath.addSetpoint(0, 0, (bot.getRobotToGoalAngle()-bot.odometry.h));
                return true;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                double[] pows = rfPath.update(bot.odometry.getAll(), bot);
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



}
