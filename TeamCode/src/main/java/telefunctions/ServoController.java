package telefunctions;

public class ServoController {
    double cur;
    double low;
    double upp;

    public ServoController(double start, double lower, double upper){
        cur = start;
        low = lower;
        upp = upper;
    }

    public double update(double val, double speed){
        speed*=0.01;
        if(val > 0){
            if(cur+speed < upp){
                cur += speed;
            }
        }else if(val < 0){
            if(cur-speed > low){
                cur -= speed;
            }
        }
        return cur;
    }
    public double update(boolean buton1, boolean buton2, double speed){
        speed*=0.01;
        if(buton1){
            if(cur+speed < upp){
                cur += speed;
            }
        }else if(buton2){
            if(cur-speed > low){
                cur -= speed;
            }
        }
        return cur;
    }


}
