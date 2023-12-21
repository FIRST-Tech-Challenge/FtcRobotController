package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Constants.Climb.*;

public class climb {
    PIDFController m_pidfController;

    private HardwareMap map;
    private MotorEx climb_motor;


    public climb(HardwareMap map, Telemetry telemetry){
        this.map=map;
        m_pidfController = new PIDFController(kp, ki, kd, kf);
        climb_motor = new MotorEx(map, "climb_motor");
    }
    public void setMotors (){
        climb_motor.set(climb_speed);
    }
    public Command climb_down(){
        return new RunCommand(()->{
            climb_motor.set(-climb_speed);
        },(Subsystem) this);
    }

}




