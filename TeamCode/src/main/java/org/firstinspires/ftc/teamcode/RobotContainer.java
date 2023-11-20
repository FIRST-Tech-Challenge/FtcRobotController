package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Command.*;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
public class RobotContainer extends com.arcrobotics.ftclib.command.Robot {

    private final gripper m_gripper;

    public RobotContainer(HardwareMap map, Telemetry telemetry){
        m_gripper = new gripper(map,telemetry);
    }
    //bind commands to trigger
    public void bindCommands(Gamepad gamepad){

        new Trigger(()->gamepad.a).whileActiveContinuous(m_gripper.openGripper());
        new Trigger(()->gamepad.b).whileActiveContinuous(m_gripper.closeGripper());
    }

    public Command AutonomousCommand(){
        return null;
    }
}
