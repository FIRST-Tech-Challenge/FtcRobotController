package org.firstinspires.ftc.teamcode.TeleOps.Memdev.Zayne;

import com.arcrobotics.ftclib.command.SubsystemBase;

public class learnMechs extends SubsystemBase{
    //Enum represents the different states of the v4b. This will be  used later when we set the potions
    public enum V4BState{
        GROUND(0.9);
        public double pos;

        V4BState(double pos) {
            this.pos = pos;
        }

    }

        
    
}
