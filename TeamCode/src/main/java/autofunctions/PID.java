package autofunctions;

public class PID {
    public double Kp = 0;
    public double Kd = 0;
    public double Ki = 0;


    public void setCoeffecients(double k, double d, double i){
        Kp = k;
        Kd = d;
        Ki = i;
    }

    public double getPower(double ce, double cv, double ci){
        return (Kp*abs(ce) - Kd*abs(cv) + Ki * abs(ci));
    }
    public double abs(double in){
        return Math.abs(in);
    }
}
