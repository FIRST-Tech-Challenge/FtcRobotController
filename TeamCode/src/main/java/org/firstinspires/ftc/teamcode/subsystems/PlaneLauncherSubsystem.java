package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
<<<<<<< Updated upstream
=======
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
>>>>>>> Stashed changes
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class PlaneLauncherSubsystem extends SubsystemBase {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    Servo m_launcherServo;
    double m_position;


<<<<<<< Updated upstream
    public static double minScale = 0.3;
    public static double maxScale = 0.97;

=======
    public static double minScale = 0.00;
    public static double maxScale = 1.00;

    public static double upPose = 0.5;
    public static double downPose = 0.0;
>>>>>>> Stashed changes

    public PlaneLauncherSubsystem(HardwareMap hardwareMap, Telemetry telemetry, double initial_position) {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
<<<<<<< Updated upstream
      //  m_launcherServo = hardwareMap.get(Servo.class, "launcher");
        m_position = initial_position;
=======
        m_launcherServo = hardwareMap.get(Servo.class, "airplane");
        m_position = downPose;
>>>>>>> Stashed changes
    }


    public void shoot() {
<<<<<<< Updated upstream
        m_position = 0.5;
    }

    public void reload() {
        m_position = 0.0;
=======
        m_position = upPose;
    }

    public void reload() {
        m_position = downPose;
>>>>>>> Stashed changes
    }

    public void actuate()
    {
        m_position = (m_position + 1.0) % 2.0;
    }
<<<<<<< Updated upstream
    public double getCassetPosition(){return m_position;}
    @Override
    public void periodic() {
       // m_launcherServo.setPosition(m_position);
       // m_launcherServo.scaleRange(minScale, maxScale);
//
       // m_telemetry.addData("launcher Pos: ", m_launcherServo.getPosition());
=======


    public Command shootCommand(){
        return new InstantCommand(()-> this.shoot());
    }

    @Override
    public void periodic() {
        m_launcherServo.setPosition(m_position);
        m_launcherServo.scaleRange(minScale, maxScale);

        m_telemetry.addData("launcher Pos: ", m_launcherServo.getPosition());
>>>>>>> Stashed changes
        m_telemetry.addData("Set Position: ", m_position);
        m_telemetry.update();
    }
}

