package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robot.DriveTrain;

@TeleOp(name = "MotorTest")
public class MotorTest extends OpMode {
    private DcMotor motor;
    private DriveTrain driveTrain;
    private ElapsedTime elapsedTime;
    boolean reset;
    double spinTime = 0;

    @Override
    public void init() {
        driveTrain = new DriveTrain(new Motor(hardwareMap, "dl"),
                new Motor(hardwareMap, "dr"),
                Motor.RunMode.RawPower);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        motor = hardwareMap.dcMotor.get("fw");
        elapsedTime = new ElapsedTime();
        reset = true;
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            if(reset) {
                reset = false;
                elapsedTime.reset();
            }
            spinTime = elapsedTime.seconds();
            driveTrain.setSpeed(1, -1);
        } else {
            reset = true;
            driveTrain.setSpeed(0, 0);
        }

        telemetry.addData("Right Speed", driveTrain.driveRight.getCorrectedVelocity());
        telemetry.addData("Right Power", driveTrain.driveRight.get());
        telemetry.addData("Elapsed Time", spinTime);
    }

}
