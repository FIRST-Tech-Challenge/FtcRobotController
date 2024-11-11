package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.Range;

public class SwerveModule {
    // drive gears, steering gears, drive motor, azimuth motor, absolute encoder

    private DcMotorEx motor;
    private CRServo servo;
    private AbsoluteAnalogEncoder enc;
    private com.arcrobotics.ftclib.controller.PIDController scontroller;
    private final double WHEEL_RAD = 2.67717; //inches might change irl due to wheel squish
    private final double DRIVE_RATIO = (52 * 2 * 2) / 18.0; //208/18
    private final double AZIMUTH_RATIO = 1.0; //for now
    private final double TPR = 28 * DRIVE_RATIO; //ticks per 1 wheel irl rotation;


    public SwerveModule(DcMotorEx m, CRServo s, AbsoluteAnalogEncoder e, double r, double sp, double si, double sd) {
        motor = m;
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        servo = s;

        enc = e;

        scontroller = new PIDController(sp, si, sd);

    }

    public void azimtuh(double pos) { //in RADianz
        scontroller.setSetPoint(pos);

        while (!scontroller.atSetPoint()) {
            double output = scontroller.calculate(
                    enc.getCurrentPosition(), pos  // the measured value and the setpoint
            );

            servo.setPower(output);
        }

    }

    public void drive (double pos, double x, double y) {
        
    }
}