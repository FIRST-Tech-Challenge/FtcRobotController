package telefunctions;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import globalfunctions.TerraThread;
import globalfunctions.Constants;
import util.CodeSeg;
import util.Stage;

public class AutoModule {


    public ArrayList<Stage> stages = new ArrayList<>();
    public int stageNum = 0;

    public ElapsedTime timer = new ElapsedTime();


    public boolean pausing = false;


    public CodeSeg updateCode = new CodeSeg() {
        @Override
        public void run() {
            Stage s = stages.get(stageNum);
            if (s.run(timer.seconds())) {
                stageNum+=1;
                timer.reset();
            }
            if (stageNum == (stages.size())) {
//                stop();
                stageNum = 0;
            }
        }
    };

    public TerraThread autoModuleThread;
    public boolean inited = false;

    public void start(){
        if(!inited) {
            autoModuleThread = new TerraThread(updateCode);
            autoModuleThread.changeRefreshRate(Constants.AUTOMODULE_REFRESH_RATE);
            inited = true;
            Thread t = new Thread(autoModuleThread);
            t.start();
        }else{
            if (!autoModuleThread.executing) {
                autoModuleThread = new TerraThread(updateCode);
                autoModuleThread.changeRefreshRate(Constants.AUTOMODULE_REFRESH_RATE);
                Thread t = new Thread(autoModuleThread);
                t.start();
            }
        }
        pausing = false;
    }

    public boolean isExecuting(){
        return autoModuleThread.executing;
    }
    public void stop(){
        autoModuleThread.stop();
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

    public void addOuttake(final DcMotorEx outr, final DcMotorEx outl, final double outrVel, final double outlVel) {
        if(outrVel == 0){
            stages.add(new Stage() {
                @Override
                public boolean run(double in) {
                    outr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    outl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    outr.setPower(0);
                    outl.setPower(0);
                    return true;
                }
            });
        } else {
            stages.add(new Stage() {
                @Override
                public boolean run(double in) {
                    outr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    outl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    outr.setVelocity(outrVel);
                    outl.setVelocity(outlVel);
                    return true;
                }
            });
        }
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

    public void addWait(final double t) {
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                return in > t;
            }
        });
    }
}
