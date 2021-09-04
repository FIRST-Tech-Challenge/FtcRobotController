package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//Created by Ethan Sanders
public class LifterCode {
    public static class Lifter {
        public DcMotor LiftMotor;
        public void MoveLift(double Power) {
            if (Power == 0) {
                LiftMotor.setPower(-0.1);
            } else if (Power < 0) {
                LiftMotor.setPower(Power * -0.46);
            } else {
                LiftMotor.setPower(-Power);
            }
        }
        public void Stop() {
            this.MoveLift(0);
        }

    }
}
