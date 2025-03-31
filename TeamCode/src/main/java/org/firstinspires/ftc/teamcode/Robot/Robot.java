package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.AutomaticPivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.HockeyStick;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtendo;

public class Robot {
    
    DriveTrain driveTrain;
    Claw claw;
    HockeyStick hockeyStick;
    HorizontalExtendo horizontalExtendo;
    AutomaticPivot automaticPivot;

    enum ScoringType{
        Parallel,
        Perpendicular
    }
    ScoringType currentState;
    public Robot(HardwareMap hw)
    {
        this.driveTrain = new DriveTrain(hw,"frontLeft", "backLeft", "frontRight",
                "backRight");
        this.claw = new Claw(hw,"Claw");
        this.hockeyStick =  new HockeyStick(hw, "hockeyStick");
        this.horizontalExtendo = new HorizontalExtendo(hw, "leftExtendo", "rightExtendo");
        this.automaticPivot =  new AutomaticPivot(hw);
        currentState = ScoringType.Parallel;
    }

    public void drive(double forward, double strafe, double rotate)
    {
        driveTrain.drive(forward, strafe, rotate);
    }
    public void initializeStates()
    {
        automaticPivot.init();
        claw.open();
    }

    public void resetStates()
    {
        automaticPivot.setModeParallel();
        horizontalExtendo.resetPos();

    }

    public void intakeToggle()
    {
        automaticPivot.toggleMode();
    }

    public void toggleClaw()
    {
        claw.toggle();
    }

    public void ringClaw()
    {
        claw.ringClose();
    }

    public void updatePos()
    {
        automaticPivot.updatePos();
    }

    public void extend(double val)
    {
        horizontalExtendo.movePos(val);
    }

    public void toggleHockeyStick()
    {
        hockeyStick.toggle();
    }

    public void adjustPivotOffset(int pivotAdjustment)
    {
       automaticPivot.adjustOffset(pivotAdjustment);
    }

    public void adjustHockeyOffset(double hockeyOffset)
    {
        hockeyStick.adjustPos(hockeyOffset);
    }


}
