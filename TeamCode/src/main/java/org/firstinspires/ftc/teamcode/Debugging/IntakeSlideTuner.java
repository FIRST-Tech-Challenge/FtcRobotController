package org.firstinspires.ftc.teamcode.Debugging;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.IntakeConstants;

@Config
@TeleOp
public class IntakeSlideTuner extends OpMode {

    public static double
            p = IntakeConstants.sp,
            i = IntakeConstants.si,
            d = IntakeConstants.sd;

    public static double target;

    private PIDController controller = new PIDController(p,i, d);

    private Hardware hardware = new Hardware();



    @Override
    public void init() {
        hardware.init(hardwareMap);
    }

    @Override
    public void loop() {
        hardware.clearCache();
        double pos = hardware.intakeSlide.getCurrentPosition();

        double output = controller.calculate(pos, target);

        hardware.intakeSlide.setPower(output);

        telemetry.addData("Target Pos: ", target);
        telemetry.addData("Current Pos: ", pos);
        telemetry.addData("Output: ", output);
        telemetry.addData("Current: ", hardware.intakeSlide.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}
