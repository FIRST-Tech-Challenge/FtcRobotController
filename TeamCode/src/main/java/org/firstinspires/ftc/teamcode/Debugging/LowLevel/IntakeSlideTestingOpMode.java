package org.firstinspires.ftc.teamcode.Debugging.LowLevel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Constants.IntakeConstants;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class IntakeSlideTestingOpMode extends OpMode {

    public static double
            p = IntakeConstants.sp,
            i = IntakeConstants.si,
            d = IntakeConstants.sd;

    public static double target = 0;
    private double prevTarget = 0;

    private PIDController controller = new PIDController(p,i, d);

    private Hardware hardware = new Hardware();

    private DcMotorEx slideMotor;

    private Timing.Timer timer = new Timing.Timer(999999999, TimeUnit.MILLISECONDS);



    @Override
    public void init() {
        hardware.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideMotor = hardware.intakeSlideMotor;
    }

    public void start() {
        timer.start();
    }

    @Override
    public void loop() {
        hardware.clearCache();
        double pos = hardware.intakeSlideMotor.getCurrentPosition();
        controller.setPID(p,i, d);
        double output = controller.calculate(pos, target);

        hardware.intakeSlideMotor.setPower(output);

        if (target != prevTarget) {
            timer.start();
        }

        if (atPos(pos, target)) {
            timer.pause();
        }

        prevTarget = target;

        telemetry.addData("Target Pos: ", target);
        telemetry.addData("Current Pos: ", pos);
        telemetry.addData("Output: ", output);
        telemetry.addData("Current: ", hardware.intakeSlideMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Cycle Time: ", controller.getPeriod());
        telemetry.addData("Elapsed Time (ms): ", timer.elapsedTime());
        telemetry.update();
    }

    public boolean atPos(double current, double target){
        return (Math.abs(target - current) < 10);
    }

}
