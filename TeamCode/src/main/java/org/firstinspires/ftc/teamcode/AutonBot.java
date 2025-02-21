package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.bots.FSMBot;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
public class AutonBot extends FSMBot {

    public AutonBot(LinearOpMode opMode) {
        super(opMode);
    }

    public void UpdateStates(){
        super.onTick();
    }

    @Override
    protected void onTick() {
        super.onTick();
    }
}
