package global;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public interface RobotPart {
    public ArrayList<DcMotor> motors = new ArrayList<>();
    public void init();



}
