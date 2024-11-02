package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveDistanceCmd;
import org.firstinspires.ftc.teamcode.commands.TurnCmd;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;
import org.firstinspires.ftc.teamcode.subsystems.ImuSub;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "Tele-op 2024-25")
public class MacroRecorder extends CommandOpMode {

    private GamepadEx driverOp;
    private GamepadEx toolOp;
    private DrivetrainSub drive;
    private DriveDistanceCmd driveCmd;
    private TurnCmd turnCW;
    private TurnCmd turnCCW;
    private boolean fieldCentric = false;
    private ImuSub robotImu;

    private ButtonReader DPAD_UP;
    private ButtonReader DPAD_LEFT;
    private ButtonReader DPAD_RIGHT;

    private List<String> moves;


    @Override
    public void initialize() {
        robotImu = new ImuSub(hardwareMap, telemetry);

        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        drive = new DrivetrainSub(hardwareMap, telemetry);
        driveCmd = new DriveDistanceCmd(0.5, 0.5, drive, telemetry);
        turnCW = new TurnCmd(-5, 0.4, drive, robotImu, telemetry);
        turnCCW = new TurnCmd(5, 0.4, drive, robotImu, telemetry);

        ButtonReader DPAD_UP = new ButtonReader(driverOp, GamepadKeys.Button.DPAD_UP);
        ButtonReader DPAD_LEFT = new ButtonReader(driverOp, GamepadKeys.Button.DPAD_LEFT);
        ButtonReader DPAD_RIGHT = new ButtonReader(driverOp, GamepadKeys.Button.DPAD_RIGHT);

        moves.set(0, "BEGIN");


        register(drive);
        //register(linearSlide);
        drive.setDefaultCommand(driveCmd);
        //linearSlide.setDefaultCommand(linearSlideCmd);

    }


    @Override
    public void run() {
        super.run();

        if (DPAD_UP.wasJustPressed()) {
            driveCmd.execute();
            moves.add("FW");
        } else if (DPAD_RIGHT.wasJustPressed()) {
            turnCW.execute();
            moves.add("CW");
        } else if (DPAD_LEFT.wasJustPressed()) {
            turnCCW.execute();
            moves.add("CCW");
        }

        ButtonReader br = new ButtonReader(driverOp, GamepadKeys.Button.A);
        if (br.wasJustPressed()) {
            List<String> newmoves = new ArrayList<String>();
            String last_move = "";
            double count = 0;
            for (String move : moves) {
                if (move.equals(last_move)) {
                    if (move.equals("FW")) {
                        count += 0.5;
                    } else if (move.equals("CW")) {
                        count -= 5;
                    } else if (move.equals("CCW")) {
                        count += 5;
                    }
                } else {
                    newmoves.add(last_move + " " + count);
                    count = 0;
                    last_move = move;
                }
            }
            for (String newmove : newmoves) {
                telemetry.addData(newmove, "");
            }
        }


        telemetry.addData("Field Centric?", fieldCentric);
        telemetry.update();
    }

    public boolean getFieldCentric() {
        return fieldCentric;
    }

    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
        if (fieldCentric) {
            robotImu.resetAngle();
        }
    }
}
