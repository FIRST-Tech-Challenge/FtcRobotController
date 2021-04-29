package telefunctions;

//Servo Controller used to move servos smoothly from pos to pos
public class ServoController {
    public double cur;
    double low;
    double upp;
    //Constructor that inputs start pos, lower and upper bounds
    public ServoController(double start, double lower, double upper){
        cur = start;
        low = lower;
        upp = upper;
    }
    //updates Servocontroler with val1 increasing pos and val2 decreasing
    public double update(double val1, double val2, double speed){
        double val =  val1 - val2;
        speed *= Math.signum(val) * 0.01;
        if (cur+speed < upp && cur+speed > low) {
            cur += speed;
        }
        return cur;
    }

    //updates Servocontroller with button 1 increasing and button 2 decreasing
    public double update(boolean button1, boolean button2, double speed){
        speed *= 0.01 * (((button1) ? 1 : 0) - ((button2) ? 1 : 0));
        if (cur+speed < upp && cur+speed > low) {
            cur += speed;
        }
        return cur;
    }


}
