package telefunctions;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class Limits {
    ArrayList<DcMotor> motors = new ArrayList<>();
    ArrayList<Double> lower = new ArrayList<>();
    ArrayList<Double> upper = new ArrayList<>();


    public void addLimit(DcMotor d, double low, double high) {
        motors.add(d);
        lower.add(low);
        upper.add(high);
    }

    public boolean isInLimits(DcMotor d, double dir,  double pos) {
        int i = motors.indexOf(d);
        double lower_bound = lower.get(i);
        double upper_bound = upper.get(i);

        return !((lower_bound >= pos && dir < 0) || (pos >= upper_bound && dir > 0));
    }


}