package telefunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


public class Cycle {
    ElapsedTime delay = new ElapsedTime();
    ArrayList<Double> poses = new ArrayList<>();
    public int curr = 0;
    public Cycle(double p1){
        poses.add(p1);
    }
    public Cycle(double p1, double p2){
        poses.add(p1);
        poses.add(p2);
    }
    public Cycle(double p1, double p2, double p3){
        poses.add(p1);
        poses.add(p2);
        poses.add(p3);
    }
    public double getPos(int index){
        return poses.get(index);
    }
    public void addPos(double pos){
        poses.add(pos);
    }
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
