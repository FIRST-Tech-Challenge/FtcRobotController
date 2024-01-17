package org.firstinspires.ftc.teamcode.subsystems;

<<<<<<< Updated upstream
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
=======
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
>>>>>>> Stashed changes
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
<<<<<<< Updated upstream
=======
import com.qualcomm.robotcore.hardware.Servo;
>>>>>>> Stashed changes
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.sun.source.doctree.StartElementTree;

import org.firstinspires.ftc.robotcore.external.Telemetry;

<<<<<<< Updated upstream
public class IntakeSubsystem extends SubsystemBase {
    DcMotorEx flyWheel;
    DcMotorEx flyWheelOutside;
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
=======
import java.util.function.DoubleSupplier;
@Config
public class IntakeSubsystem extends SubsystemBase {
    DcMotorEx flyWheel;
    DcMotorEx flyWheelOutside;
    Servo m_servo;
    public String intakeState = "intake";
    public String stowState = "stow";
    public double m_position = stowPose;
    public static double groundPose = 0.95;
    public static double topPose = 0.77;
    public static double middlePose = 0.81;
    public static double lowPose = 0.855;
    public static double lowlowPose = 0.885;
    public static double stowPose = 0.47;
    public static double intakePose = 0.95;
    public String desiredState = stowState;
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    public static double speed = 1.0;

>>>>>>> Stashed changes
    public static double P =9;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
<<<<<<< Updated upstream
=======
    double THROTTLEMINLEVEL = 0.4;
    public int decision = 5;

    public static double minScale = 0.0;
    public static double maxScale = 1.0;

    public static double slowSpeed = 1.0;
    public static double fastSpeed = 1.0;
>>>>>>> Stashed changes

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        flyWheel = m_hardwareMap.get(DcMotorEx.class, "flyWheel");
<<<<<<< Updated upstream
        flyWheelOutside = m_hardwareMap.get(DcMotorEx.class, "pe");

        flyWheelOutside.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheelOutside.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorConfigurationType motorConfigurationType = flyWheelOutside.getMotorType();
        motorConfigurationType.setAchieveableMaxRPMFraction(0.99);
        motorConfigurationType.setMaxRPM(435);
=======

 //       flyWheelOutside = m_hardwareMap.get(DcMotorEx.class, "pe");

//        flyWheelOutside.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        flyWheelOutside.setDirection(DcMotorSimple.Direction.REVERSE);
//        MotorConfigurationType motorConfigurationType = flyWheelOutside.getMotorType();
//        motorConfigurationType.setAchieveableMaxRPMFraction(0.99);
//        motorConfigurationType.setMaxRPM(435);

        m_servo = m_hardwareMap.get(Servo.class, "FI");

>>>>>>> Stashed changes

        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorConfigurationType motorConfigurationType2 = flyWheel.getMotorType();
        motorConfigurationType2.setAchieveableMaxRPMFraction(0.99);
        motorConfigurationType2.setMaxRPM(435);
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
    }

    public void stop(){
        flyWheel.setPower(0);
<<<<<<< Updated upstream
        flyWheelOutside.setPower(0);
        }


    public void intake(double speed){
        flyWheel.setPower(speed);
        flyWheelOutside.setPower(speed);

    }
    public void outake(){
        flyWheel.setPower(-0.7);
        flyWheelOutside.setPower(-0.7);

    }

    public Command intakeCommand(){
        return new InstantCommand(()-> this.intake(1.0));
    }

=======

      //  flyWheelOutside.setPower(0);
        }


    public void intake(DoubleSupplier intakeThrottle){
        double throttleSlope = 1 - THROTTLEMINLEVEL;
        double throttleScale = throttleSlope * intakeThrottle.getAsDouble() + THROTTLEMINLEVEL;
        flyWheel.setPower(1.0*throttleScale);
       // flyWheelOutside.setPower(speed);

    }

    public void intake(Double speed){

        flyWheel.setPower(speed);
        // flyWheelOutside.setPower(speed);

    }


    public void changeState(){
        if(desiredState == intakeState){
            desiredState = stowState;
        }
        else if(desiredState == stowState){
            desiredState = intakeState;
        }

    }

    public void setState(String state){
        desiredState = state;
    }


    public void setFoldablePose(double x, double y){
        if(x > 0.7 && y < 0.25 && y > -0.25){
            decision = 4//
            ;
        }
        else if(y < -0.75 && x < 0.25 && x > -0.25){
            decision = 1;//
        } else if(y > 0.75 && x < 0.25 && x > -0.25){
            decision = 3;//
        } else if(x < -0.7 && y< 0.25 && y > -0.25){
            decision = 2;//
        }  else if(x < 0.25 && x > -0.255 && y< 0.25 && y > -0.25)
        {decision = 0;}

        switch(decision) {
            case 0:
                changePose(0);
                break;
            case 1:
                changePose(1);
                break;
            case 2:
                changePose(2);
                break;
            case 3:
                changePose(3);
                break;
            case 4:
                changePose(4);
                break;
        }
    }
    public void changePose(int mpose){
        decision = mpose;
        if(decision == 3) {
            decision = 3;
            intakePose = middlePose;
        }
        if(decision == 4) {
            decision = 4;
            intakePose = topPose;
        }
        if(decision == 2) {
            decision = 2;
            intakePose = lowPose;
        }
        if(decision == 1) {
            decision = 1;
            intakePose = lowlowPose;
        }
        if(decision <= 0){
            decision = 0;
            intakePose = groundPose;
        }
    }





    public void outake(){
        flyWheel.setPower(-1.0);
      //  flyWheelOutside.setPower(-0.3);

    }

    public Command intakeCommand(DoubleSupplier throttle){
        return new InstantCommand(()-> this.intake(throttle));
    }

    public Command intakeCommand(double speed){
        return new InstantCommand(()-> this.intake(speed));
    }

    public Command stateChangeCommand(){
        return new InstantCommand(()-> this.changeState());
    }

    public Command changePoseCommand(int pose){return new InstantCommand(()-> this.changePose(pose));}

    public Command intakeStackCommand(){

        int decision2 = decision -1;
        return
            new SequentialCommandGroup(

                    new InstantCommand(()-> this.changePose(decision)),
                    new WaitCommand(500),
                    new InstantCommand(()-> this.changePose(decision2)));}


    public Command setStateCommand(String dState){return new InstantCommand(()-> this.setState(dState));}
//    public Command setPoseCommand(int pose){
//        return new InstantCommand(()-> this.setFoldablePose(pose));
//    }

>>>>>>> Stashed changes
    public Command outakeCommand(){
        return new InstantCommand(()-> this.outake());
    }

    public Command stopCommand(){return new InstantCommand(()-> this.stop());}
<<<<<<< Updated upstream
    public void periodic(){
        flyWheel.setVelocityPIDFCoefficients(P, I, D, F);
        m_telemetry.addData("desired velocity", flyWheel.getVelocity());

=======


    public void periodic(){

        flyWheel.setVelocityPIDFCoefficients(P, I, D, F);
        m_telemetry.addData("desired velocity", flyWheel.getVelocity());
        if(desiredState == intakeState){m_position = intakePose;}
        if(desiredState == stowState){m_position = stowPose;}
        m_servo.setPosition(m_position);
        m_servo.scaleRange(minScale, maxScale);
>>>>>>> Stashed changes
    }

}
