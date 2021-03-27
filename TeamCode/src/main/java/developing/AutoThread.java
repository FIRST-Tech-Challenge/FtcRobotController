package developing;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import telefunctions.Stage;

public class AutoThread implements Runnable{
        public boolean executing = false;
        public int refreshRate = 100; // hertz



        public void init(){
            executing = true;
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

        }

        @Override
        public void run() {
            while (isExecuting()) {
                update();
                try {Thread.sleep(1000/refreshRate); } catch (InterruptedException e) {}
            }
        }


}
