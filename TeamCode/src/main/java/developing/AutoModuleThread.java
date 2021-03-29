package developing;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import telefunctions.Stage;

public class AutoModuleThread implements Runnable{
        public boolean executing = false;
        public int stageNum = 0;
        public ArrayList<Stage> stages = new ArrayList<>();

        public ElapsedTime timer = new ElapsedTime();


        public int refreshRate = 100; // hertz



        public void init(ArrayList<Stage> stages){
            this.stages = stages;
            executing = true;
            timer.reset();
        }

        public void changeRefreshRate(int rf){
            refreshRate = rf;
        }

        public synchronized void stop() {
            this.executing = false;
        }

        private synchronized boolean isExecuting() {
            return this.executing;
        }

        public void update() {
            Stage s = stages.get(stageNum);
            if (s.run(timer.seconds())) {
                stageNum+=1;
                timer.reset();
            }
            if (stageNum == (stages.size())) {
                stop();
                stageNum = 0;
            }
        }

        @Override
        public void run() {
            while (isExecuting()) {
                update();
                try {Thread.sleep(1000/refreshRate); } catch (InterruptedException e) {}
            }
        }


}
