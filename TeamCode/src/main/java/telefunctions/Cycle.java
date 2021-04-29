package telefunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


//Used to cycle between servo positions
public class Cycle {
    ElapsedTime delay = new ElapsedTime();
    ArrayList<Double> poses = new ArrayList<>();
    public int curr = 0;
    //Define cycle with positins and inputs
    public Cycle(double ...p){
        for(double i:p){
            poses.add(i);
        }
    }
    //Changes the current index to the specified value
    public void changeCurr(int cur){
        curr = cur;
    }
    //Gets the position in poses at a certain index
    public double getPos(int index){
        return poses.get(index);
    }

    //Updates the cycle with double down and double up
    public double update(double down, double up){
        if(down > 0){
            if(curr > 0 && delay.seconds() > 0.3){
                curr -=1;
            }
            delay.reset();
        }else if(up > 0){
            if(curr < (poses.size()-1) &&  delay.seconds() > 0.3){
                curr+=1;
            }
            delay.reset();
        }
        return poses.get(curr);
    }
    //Updates the cycle with boolean down and boolean up
    public double update(boolean down, boolean up){
        if(down){
            if(curr > 0 && delay.seconds() > 0.1){
                curr -=1;
            }
            delay.reset();
        }else if(up){
            if(curr < (poses.size()-1) &&  delay.seconds() > 0.1){
                curr+=1;
            }
            delay.reset();
        }
        return poses.get(curr);
    }
}
