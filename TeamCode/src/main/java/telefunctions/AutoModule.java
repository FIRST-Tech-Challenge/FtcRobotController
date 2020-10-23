package telefunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class AutoModule {
    public boolean executing = false;
    public int stageNum = 0;
    public ArrayList<Stage> stages = new ArrayList();
    public ArrayList<Double> dynamics = new ArrayList();

    public ElapsedTime timer = new ElapsedTime();

    public double lastTime = 0;

    public void start() {
        executing = true;
        timer.reset();
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
    public void addStage(final DcMotor mot, final double pos, double t) {
        lastTime += t;
        final double time = lastTime;
        stages.add(new Stage() {
            @Override
            public boolean run(double in) {
                mot.setPower(pos);
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


}