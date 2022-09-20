package Framework;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Framework.subsystems.Drivetrain;

public class Robot {

    private Drivetrain drivetrain;

    public Robot(HardwareMap hw){
        drivetrain = new Drivetrain(hw);
    }


}
