//package org.firstinspires.ftc.teamcode.powerplayV2.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.FrontSliderSubsystem;
//
//import java.util.function.BooleanSupplier;
//
//public class FrontSliderRetractCommand extends CommandBase {
//    private FrontSliderSubsystem frontSlider;
//
//    private BooleanSupplier rightEndSup, leftEndSup;
//    private boolean right = false, left = false;
//
//    public FrontSliderRetractCommand(FrontSliderSubsystem frontSlider) {
//        this.frontSlider = frontSlider;
//        rightEndSup = frontSlider.rightEnd();
//        leftEndSup = frontSlider.leftEnd();
//    }
//
//    @Override
//    public void initialize() {
//        frontSlider.closeRaw();
//    }
//
//    @Override
//    public boolean isFinished() {
//        right = rightEndSup.getAsBoolean() ? true : right;
//        left = leftEndSup.getAsBoolean() ? true : left;
//
//        return right && left;
//    }
//}
