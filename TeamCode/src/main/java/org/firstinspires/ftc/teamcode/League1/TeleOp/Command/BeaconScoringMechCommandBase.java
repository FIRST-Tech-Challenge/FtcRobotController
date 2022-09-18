package org.firstinspires.ftc.teamcode.League1.TeleOp.Command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.League1.Subsystems.EndgameSystems;

public class BeaconScoringMechCommandBase extends CommandBase {

    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    Gamepad gamepad2;
    EndgameSystems endgameSystem;


    public BeaconScoringMechCommandBase(Gamepad gamepad2, HardwareMap hardwareMap){
        this.gamepad2 = gamepad2;
        endgameSystem = new EndgameSystems(hardwareMap);

    }

    @Override
    public void execute() {
        double yPos = endgameSystem.getYCapPosition();

        endgameSystem.setXCapstoneRotatePower(-gamepad2.right_stick_x / 3);
                        /*
                        if(gamepad1.right_stick_y > 0.1){
                            endgameSystem.jankUpY();
                        }else if(gamepad1.right_stick_y < -0.1){
                            endgameSystem.jankDownY();
                        }
                         */

        endgameSystem.setYCapPosition(yPos - endgameSystem.map(gamepad2.right_stick_y, -1, 1, -0.0010, 0.0010));

        if(gamepad2.right_trigger > 0.1){
            endgameSystem.setCapstoneExtensionPower(-gamepad2.right_trigger);
        }else{
            endgameSystem.setCapstoneExtensionPower(0);
        }

        if(gamepad2.left_trigger > 0.1){
            endgameSystem.setCapstoneExtensionPower(gamepad2.left_trigger);
        }else{
            endgameSystem.setCapstoneExtensionPower(0);
        }

    }


}
