package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class DoubleCassetSubsystem extends SubsystemBase{
        HardwareMap m_hardwareMap;
        Telemetry m_telemetry;
        Servo m_cassetLeftServo;
        Servo m_cassetRightServo;
        double m_leftPosition;
        double m_rightPosition;
        public static double openPoseR = 0.7;
        public static double closePoseR = 0.5;

        public static double openPoseL = 0.7;
        public static double closePoseL = 0.5;

        public static double minScale = 0.0;
        public static double maxScale = 1.0;


        public DoubleCassetSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
            m_hardwareMap = hardwareMap;
            m_telemetry = telemetry;
            m_cassetLeftServo =m_hardwareMap.get(Servo.class, "cassetLeft");
            m_cassetRightServo = m_hardwareMap.get(Servo.class, "cassetRight");
            m_leftPosition = closePoseL;
            m_rightPosition = closePoseR;

        }


        public void depositBoth() {
            m_rightPosition = openPoseR;
            m_leftPosition = openPoseL;

        }

        public void depositRight(){
            m_rightPosition = openPoseR;
        }

        public void depositLeft(){
            m_leftPosition = openPoseL;
        }




        public void intakePosition() {
            m_rightPosition = closePoseR;
            m_leftPosition = closePoseL;
        }

        public Command depositBothCommand(){
            return new InstantCommand(()-> this.depositBoth());
        }

         public Command depositRightCommand(){return new InstantCommand(()-> this.depositRight());}

          public Command depositLeftPosition(){return new InstantCommand(()-> this.depositLeft());}

        public Command intakePoseCommand(){return new InstantCommand(()-> this.intakePosition());}


        @Override
        public void periodic() {
        m_cassetRightServo.setPosition(m_rightPosition);
            m_cassetLeftServo.setPosition(m_leftPosition);
        m_cassetLeftServo.scaleRange(minScale, maxScale);
        m_cassetRightServo.scaleRange(minScale,maxScale);
//        m_telemetry.addData("Servo Pos: ", m_cassetServo.getPosition());
//        m_telemetry.addData("Set Position: ", m_position);
//        m_telemetry.update();
        }
    }


