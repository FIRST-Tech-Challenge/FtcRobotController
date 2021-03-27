package developing;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class Optimizer {
    public ArrayList<Double> times = new ArrayList<>();
    public ElapsedTime timer = new ElapsedTime();
    public double lastTime = 0;
    public double deltaTime = 0;
    public double avgDeltaTime = 0;

    public boolean show = false;


    public void update(){
        if(!show) {
            deltaTime = timer.seconds() - lastTime;
            times.add(deltaTime);
            lastTime += deltaTime;
        }
    }

    public void show(){
        show = true;
        for(double i:times){
            avgDeltaTime += (i/times.size());
        }
    }

    public void reset(){
        lastTime = timer.seconds();
        deltaTime = 0;
        avgDeltaTime = 0;
        show = false;
        times = new ArrayList<>();
    }


}
