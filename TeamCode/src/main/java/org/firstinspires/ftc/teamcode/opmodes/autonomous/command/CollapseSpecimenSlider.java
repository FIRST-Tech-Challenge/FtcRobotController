package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.specimen.SpecimenSlider;

public class CollapseSpecimenSlider extends SounderBotCommandBase {

    public CollapseSpecimenSlider(SpecimenSlider slider, long timeout, Telemetry telemetry) {
        super(timeout);

        this.slider = slider;
        this.telemetry = telemetry;
    }

    private static final long ExtensionTime = 2000;

    SpecimenSlider slider;
    Telemetry telemetry;

    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    boolean firstExecute = true;

    @Override
    public void initialize() {
    }

    @Override
    protected boolean isTargetReached() {
        return false;
    } // Keep the slider up

    long startTime;

    boolean finished = false;

    @Override
    public void doExecute() {
        if(firstExecute) {
            firstExecute = false;
            elapsedTime.reset();
            telemetry.addLine("Collapsing");
            telemetry.update();

            slider.Collapse();
        }

        telemetry.addData("elapsed", elapsedTime.milliseconds());
        telemetry.update();

        if(elapsedTime.milliseconds() > 3000 && !finished) {
            finished = true;
            slider.Hold();
            telemetry.addLine("Holding");
            telemetry.update();
        }
    }

    @Override
    protected void onTimeout() {
        super.onTimeout();
        slider.Hold();
    }
}
