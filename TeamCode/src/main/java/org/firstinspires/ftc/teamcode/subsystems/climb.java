package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import static org.firstinspires.ftc.teamcode.Constants.Climb.*;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class climb {

//    TrapezoidProfile profile =,    // Creates a new TrapezoidProfile | Profile will have a max vel of 5 meters per second | Profile will have a max acceleration of 10 meters per second squared | Profile will end stationary at 5 meters | Profile will start stationary at zero position
//            new TrapezoidProfile.State(5, 0),
//            new TrapezoidProfile.State(0, 0));

    PIDFController m_pidfController;
    TrapezoidProfile.Constraints m_sysConstraints;
    TrapezoidProfile m_profile;
    ElapsedTime m_elapsedTime;

    private HardwareMap map;
    private MotorEx climb_motor;
    private boolean isUp;


    public climb(HardwareMap map, Telemetry telemetry) {
        this.map = map;
        m_elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        m_pidfController = new PIDFController(kp, ki, kd, kf);
        climb_motor = new MotorEx(map, "climb_motor");
        m_sysConstraints = new TrapezoidProfile.Constraints(climb_max_speed, climb_max_accel);
        isUp = false;
    }

    public void stopMotors() {
        climb_motor.set(0);
    }
    public Command climb_down(){
        TrapezoidProfile.State goal_down = new TrapezoidProfile.State(0, 0);
        if((climb_motor.encoder.getPosition() <= 0)){
            stopMotors();
        }
        isUp = false;
        return climb(goal_down);
    }
    public Command climb_up() {
        TrapezoidProfile.State goal_up = new TrapezoidProfile.State(max_ticks, 0);
        if((climb_motor.encoder.getPosition() >= max_ticks)){
            stopMotors();
        }
        isUp = true;
        return climb(goal_up);
    }

    public Command climb(TrapezoidProfile.State goal) {

        Runnable init = () -> {
            TrapezoidProfile.State initPose = new TrapezoidProfile.State(climb_motor.encoder.getPosition(),
                    climb_motor.encoder.getRawVelocity());
            m_profile = new TrapezoidProfile(m_sysConstraints,goal , initPose);
            m_elapsedTime.reset();
        };
        Runnable loop = () -> {
            climb_motor.set(m_pidfController.calculate(
                    climb_motor.encoder.getPosition(),
                    m_profile.calculate(m_elapsedTime.time()).position
            ));
        };

        BooleanSupplier atGoal = () -> climb_motor.encoder.getPosition() == goal.position;
        Consumer<Boolean> stopMotors = interrupt -> stopMotors();
        return new FunctionalCommand(init, loop, stopMotors, atGoal, (Subsystem) this);

    }
//    public Command toggleClimb(){
//        if(isUp){
//            climb_down();
//        }else{
//            climb_up();
//        }
//        return new RunCommand(climb_up(), climb_down());
//    }
}






