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

    public double update(double val1, double val2, double speed){
        double val =  val1 - val2;
        speed *= Math.signum(val) * 0.01;
        if (cur+speed < upp && cur+speed > low) {
            cur += speed;
        }
        return cur;
    }

    public double update(boolean button1, boolean button2, double speed){
        speed *= 0.01 * (((button1) ? 1 : 0) - ((button2) ? 1 : 0));
        if (cur+speed < upp && cur+speed > low) {
            cur += speed;
        }
        return cur;
    }


}
