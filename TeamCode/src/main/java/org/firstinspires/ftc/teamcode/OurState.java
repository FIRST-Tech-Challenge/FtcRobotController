

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotStatus;

public class OurState extends OpMode
{
    //public HardwareMap hardwareMap = null;
    public Boolean running = true;
    public RobotHardware robotHardware = null;
    //@Override
    public OurState() {
        //hardwareMap = hm;
    }
    
    @Override
    public void init() {
        
    }
    
    public void init(RobotHardware r) {
        telemetry.addData("Status", "Initialized");
        robotHardware = r;
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        
    }

    public void loop(RobotStatus status) {
        
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
    }
    
    public double getVariable() {
        return 0;
    }
    public void addToGoal(double variable) {
        //this page intentionally left blank.
    }

}
