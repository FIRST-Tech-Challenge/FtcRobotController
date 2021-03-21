package developing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

import global.Constants;
import telefunctions.Cycle;
import telefunctions.ServoController;
import telefunctions.Stage;
import util.CodeSeg;
import util.TerraThread;

public class AutoModule2 {
    TestThread testThread = new TestThread();


    public ArrayList<Stage> stages = new ArrayList<>();


    public boolean pausing = false;




    public void start(){
        if(!testThread.executing) {
            testThread.changeRefreshRate(Constants.AUTOMODULE_REFRESH_RATE);
            testThread.init(stages);

            Thread t = new Thread(testThread);
            t.start();
        }
        pausing = false;
    }

    public boolean isExecuting(){
        return testThread.executing;
    }
    public void stop(){
        testThread.stop();
    }

    public void addDelay(final double secs){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                return in > secs;
            }
        });
    }

    public void addStage(final DcMotor mot, final double pow, final int pos) {
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
                return true;
            }
        });

    }

    public void addSave(final Cycle c, final int idx){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                c.curr = idx;
                return true;
            }
        });
    }
    public void addSave(final ServoController c, final double pos){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                c.cur = pos;
                return true;
            }
        });
    }



    public void addStage(final DcMotor mot, final double pow, final double t) {
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                mot.setPower(pow);
                return in > t;
            }
        });
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                mot.setPower(0);
                return true;
            }
        });
    }
    public void addStage(final CRServo crs, final double pow) {
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                crs.setPower(pow);
                return true;
            }
        });
    }
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

    public void addStage(final Servo s, final double pos, final double t) {
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                s.setPosition(pos);
                return in > t;
            }
        });
    }


    public void addStage(final Servo s, final Cycle cycle, final int idx, final double t) {
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                cycle.curr = idx;
                s.setPosition(cycle.getPos(idx));
                return in > t;
            }
        });
    }

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

    public void addWait(final double t){
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                return in > t;
            }
        });
    }
}
