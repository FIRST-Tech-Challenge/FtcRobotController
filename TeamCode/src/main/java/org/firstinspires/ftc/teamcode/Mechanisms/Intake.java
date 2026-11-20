package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor FrontIntake,BackIntake;
    private CRServo PollenServo, NectarServo;
    public void init(HardwareMap hwMap)
        {
            NectarServo = hwMap.get(CRServo.class,"servo_lift_nectar");
            PollenServo = hwMap.get(CRServo.class, "servo_lift_pollen");
            FrontIntake = hwMap.get(DcMotor.class, "motor_intake_front");
            BackIntake = hwMap.get(DcMotor.class, "motor_intake_back");
        }


        public void setPollenServoSpeed(double speed)
        {
            PollenServo.setPower(speed);
        }
        public void setNectarServoSpeed(double speed)
        {
            NectarServo.setPower(speed);
        }
        public void setFrontIntakeMotorSpeed(double speed)
        {
            //accepts values from -1.0 to 1.0
            FrontIntake.setPower(speed);
        }
        public void setbackIntakeMotorSpeed(double speed)
        {
            //accepts values from -1.0 to 1.0
            BackIntake.setPower(speed);
        }


}
