package telefunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import autofunctions.Path;
import global.TerraBot;
import util.CodeSeg;

public class AutoModule {
    public boolean executing = false;
    public int stageNum = 0;
    public ArrayList<Stage> stages = new ArrayList();

    public ElapsedTime timer = new ElapsedTime();

    public double lastTime = 0;

    public boolean pausing = false;


    public void start() {
        executing = true;
        timer.reset();
        lastTime = 0;
        pausing = false;
    }

    public void update() {
        if (executing) {
            Stage s = stages.get(stageNum);
            if (s.run(timer.seconds())) {
                stageNum+=1;
            }
            if (stageNum == (stages.size())) {
                executing = false;
                stageNum = 0;
            }
        }
    }

    public void addWaitUntil(){
        lastTime = 0;
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                executing = false;
                pausing = true;
                return true;
            }
        });
    }
    public void addDelay(final double secs){
        lastTime += secs;
        final double time = lastTime;
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                return in > time;
            }
        });
    }

    public void addStage(final DcMotor mot, final double pow, final int pos) {
        lastTime = 0;
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                mot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mot.setTargetPosition(pos);
                mot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mot.setPower(pow);
                return true;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
               return !mot.isBusy();
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                mot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mot.setPower(0);
                timer.reset();
                lastTime = 0;
                return true;
            }
        });

    }

    public void addSave(final Cycle c, final int idx){
        lastTime = 0;
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                timer.reset();
                c.curr = idx;
                return true;
            }
        });
    }
    public void addSave(final ServoController c, final double pos){
        lastTime = 0;
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                timer.reset();
                c.cur = pos;
                return true;
            }
        });
    }



    public void addStage(final DcMotor mot, final double pow, final double t) {
        lastTime += t;
        final double time = lastTime;
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                mot.setPower(pow);
                return in > time;
            }
        });
    }

    public void addStage(final Servo s, final double pos, double t) {
        lastTime += t;
        final double time = lastTime;
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                s.setPosition(pos);
                return in > time;
            }
        });
    }
    public void addPath(final Path path, final TerraBot bot){
        lastTime = 0;
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                path.timer.reset();
                return true;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                double[] pows = path.update(bot.odometry.getPos(), bot.odometry.getVels());
                bot.move(pows[1], pows[0], pows[2]);
                return !path.isExecuting;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                bot.move(0,0,0);
                timer.reset();
                return true;
            }
        });
    }

    public void addCustomOnce(final CodeSeg cs){
        lastTime = 0;
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                cs.run();
                return true;
            }
        });
    }

    public void addCustom(final CodeSeg cs, double t){
        lastTime += t;
        final double time = lastTime;
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                cs.run();
                return in > time;
            }
        });
    }

    public void addSPWait(final SpeedController sl, final  SpeedController sr){
        lastTime = 0;
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                timer.reset();
                return sl.isReady() && sr.isReady();
            }
        });
    }

    public void addOdometySave(final double x, final double y, final TerraBot bot){
        lastTime = 0;
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
               bot.odometrySave(x,y);
                timer.reset();
                return true;
            }
        });
    }




}