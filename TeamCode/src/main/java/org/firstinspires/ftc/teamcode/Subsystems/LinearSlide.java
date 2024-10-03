package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotContainer;



/** Subsystem */
public class LinearSlide extends SubsystemBase {
        private DcMotorEx leftMotor;
        private DcMotorEx rightMotor;



    // Local objects and variables here

    /** Place code here to initialize subsystem */
    public LinearSlide() {
        leftMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "left_linear_slide");
        rightMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "right_linear_slide");

             leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftMotor.setDirection(DcMotorSimple.Direction.REVERSE); //if this does not work it was jeff and anyone other then Lonan. Lonan is the smartest tbh UwU
                    rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

      leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    // place special subsystem methods here

        public void moveLinearSlide(int ticks){

                leftMotor.setTargetPosition(ticks);
                rightMotor.setTargetPosition(ticks);
     leftMotor.setPower(1);
     rightMotor.setPower(1);
    }

    public void moveTo (SlideTargetHeight target) {moveLinearSlide(target.getValue());}

}

//place notes here

    //UwU
    //Lonans first program 10/2/2024
    //Dont make this look ugly it looks good dont change it