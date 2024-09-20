package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.LocalizerInterface;

/**
 * Modified version of ThreeDeadWheelLocalizer
 * TODO: Make sure to adjust the Params in ThreeDeadWheelLocalizer.java
 */
public class ThreeEncoderLocalizer extends ThreeDeadWheelLocalizer implements LocalizerInterface {


    public double weight;
    private Pose2d position;

    public ThreeEncoderLocalizer(HardwareMap hw){
        this(hw, new String[] {"FLM","BLM","FRM"});
    }

    public ThreeEncoderLocalizer(HardwareMap hw, String[] names){
        super(hw, 4048, names);
    }
    public ThreeEncoderLocalizer(HardwareMap hardwareMap, double inPerTick) {
        super(hardwareMap, inPerTick);
    }

    @Override
    public double getWeight() {
        return weight;
    }

    @Override
    public Pose2d getPosition() {
        Twist2dDual<Time> twist = super.update();
        position = position.plus(twist.value());
        return position;
    }
}
