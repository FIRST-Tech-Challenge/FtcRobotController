package globalfunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class Optimizer {
    //List of delta times between updates
    public ArrayList<Double> times = new ArrayList<>();
    //Timer to record times
    public ElapsedTime timer = new ElapsedTime();
    //Last time
    public double lastTime = 0;
    //Change in time
    public double deltaTime = 0;
    //Average change in time
    public double avgDeltaTime = 0;
    //Should show
    public boolean show = false;

    //Update the list with delta times
    public void update(){
        if(!show) {
            deltaTime = timer.seconds() - lastTime;
            times.add(deltaTime);
            lastTime += deltaTime;
        }
    }

    //Shows the avg delta time
    public void show(){
        show = true;
        avgDeltaTime = calcAvg(times);
    }
    //Gets the average refresh rate
    public double getRefreshRate(){
        return 1/avgDeltaTime; //in htz
    }
    //Resets the timer
    public void reset(){
        lastTime = timer.seconds();
        deltaTime = 0;
        avgDeltaTime = 0;
        show = false;
        times = new ArrayList<>();
    }
    //Calculates the average of an arraylist of doubles
    public static double calcAvg(ArrayList<Double> in){
        double total = 0;
        for(double i:in){
            total += (i/in.size());
        }

        return total;
    }
    //Calculates the maximuim of an arraylist of doubles
    public static double max(ArrayList<Double> in){
        double max = 0;
        for(double i:in){
            if (i > max){
                max = i;
            }
        }
        return max;
    }
    //Checks if the quantity is within a range
    public static boolean inRange(double in, double[] range){
        return in > range[0] && in < range[1];
    }
    //Calculates the weighted average of a list
    public static double weightedAvg(double[] in, double[] weights){
        double sum = 0;
        double wsum = 0;
        for (int i = 0; i < in.length; i++) {
            sum += in[i]*weights[i];
            wsum += weights[i];
        }
        return sum/wsum;
    }
    //Optimizes heading to [-180,180] range
    public static double optimizeHeading(double heading){
        if(!inRange(heading, new double[]{-180,180})){
            double heading2 = (Math.abs(heading)%360);

            if(inRange(heading2, new double[]{0,180})){
                return Math.signum(heading)*heading2;
            }else{
                return -Math.signum(heading)*(360 - heading2);
            }
        }else{
            return heading;
        }
    }
    //Optimizes the heading from a pos
    public static double[] optimizePos(double[] in){
        return new double[]{in[0], in[1], optimizeHeading(in[2])};
    }
    //gets an array of doubles from a string
    public static double[] fromString(String string) {
        String[] strings = string.replace("[", "").replace("]", "").split(", ");
        double[] result = new double[strings.length];
        for (int i = 0; i < result.length; i++) {
            result[i] = Double.parseDouble(strings[i]);
        }
        return result;
    }
}
