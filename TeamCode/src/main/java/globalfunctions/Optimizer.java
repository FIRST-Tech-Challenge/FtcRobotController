package globalfunctions;

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
        avgDeltaTime = calcAvg(times);
    }

    public void reset(){
        lastTime = timer.seconds();
        deltaTime = 0;
        avgDeltaTime = 0;
        show = false;
        times = new ArrayList<>();
    }

    public double calcAvg(ArrayList<Double> in){
        double total = 0;
        for(double i:in){
            total += (i/in.size());
        }

        return total;
    }

    public double max(ArrayList<Double> in){
        double max = 0;
        for(double i:in){
            if (i > max){
                max = i;
            }
        }
        return max;
    }


}
