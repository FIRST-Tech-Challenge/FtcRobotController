package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Supplier;

public class BTController {

    static Gamepad gamepad;
     static double threshold = 0.1;
    public Trigger getTrigger(double threshold, Axis... axes) {
        Trigger result = new Trigger(()->false );
        for(Axis axis : axes ){
            result=result.or(getTrigger(threshold,axis));
        }
        return result;
    }
    public Trigger assignCommand(Command command, boolean cancelOnRelease,Axis...axes){
        if(cancelOnRelease){
            return getTrigger(threshold, axes).whileActiveContinuous(command);
        }else {
            return getTrigger(threshold, axes).whenActive(command);
        }
    }
    public enum Axis {
        LEFT_X(() -> Math.abs(gamepad.left_stick_x) > threshold),
        LEFT_Y(() -> Math.abs(gamepad.left_stick_y) > threshold),
        RIGHT_X(() -> Math.abs(gamepad.right_stick_x) > threshold),
        RIGHT_Y(() -> Math.abs(gamepad.right_stick_y) > threshold),
        LEFT_TRIGGER(() -> Math.abs(gamepad.left_trigger) > threshold),
        RIGHT_TRIGGER(() -> Math.abs(gamepad.right_trigger) > threshold);

        Axis(Supplier<Boolean> s) {
            supplier = s;
        }

        Supplier<Boolean> supplier;


    }

}

