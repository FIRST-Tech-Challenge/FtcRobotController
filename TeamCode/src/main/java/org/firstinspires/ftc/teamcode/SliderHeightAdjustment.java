package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@Config
@TeleOp
public class SliderHeightAdjustment extends LinearOpMode {
    MotorEx sliderMotor;

    //Max 2280
    //Middle: 620
    //High from Ground Juction: 2150
    //High 45deg 2000

    Telemetry dahsboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    public static int height = 0;
    public static double KP = 0;
    public static int posTolerance = 10;

    double last_height=height, last_KP=KP;

    @Override
    public void runOpMode() {
        sliderMotor = new MotorEx(hardwareMap, "slider", Motor.GoBILDA.RPM_312);
        sliderMotor.setRunMode(Motor.RunMode.PositionControl);

        sliderMotor.setPositionCoefficient(KP);

        sliderMotor.resetEncoder();

        waitForStart();

        while(opModeIsActive()) {
            if(KP != last_KP || height != last_height) {
                sliderMotor.setPositionCoefficient(KP);
                sliderMotor.setTargetPosition(height);

                last_KP = KP;
                last_height = height;
            }

            sliderMotor.set(0.4);

            sliderMotor.setPositionTolerance(posTolerance);

            dahsboardTelemetry.addData("Height: ", height);
            dahsboardTelemetry.addData("Actual Height: ", sliderMotor.getCurrentPosition());
            dahsboardTelemetry.addData("Pos Tolerance: ", height);
            dahsboardTelemetry.addData("P: ", KP);

            dahsboardTelemetry.update();
        }
    }
}
