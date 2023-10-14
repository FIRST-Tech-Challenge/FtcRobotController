package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Lift Test", group="Test")
public class LiftTest extends PPRobot {
    @Override
    public void init() {
        super.init();
        telemetry.addData("Init", "Hello Storm Trooper");
        updateTelemetry(telemetry);
    }

    @Override
    public void loop() {
        super.loop();
/*        if(getEnhancedGamepad1().isDpad_up()){
            getLiftSubsystem().extend(22d);
//            getLiftSubsystem().getLiftL().setPower(0.5);
//            getLiftSubsystem().getLiftR().setPower(0.5);
        }
        if(getEnhancedGamepad1().isDpad_down()){
            getLiftSubsystem().retract();
//            getLiftSubsystem().getLiftL().setPower(-0.2);
//            getLiftSubsystem().getLiftR().setPower(-0.2);
        }
        if(getEnhancedGamepad1().isDpad_left()){
            getLiftSubsystem().extend(5d);
//            getLiftSubsystem().getLiftL().setPower(0.0);
//            getLiftSubsystem().getLiftR().setPower(0.0);
        }
        if(getEnhancedGamepad1().isDpad_right()){
            getLiftSubsystem().extend(22d);
        }*/
//        telemetry.addLine("Lift State: " + getLiftSubsystem().getState());
//        telemetry.addLine("Encoder Ticks Left: " + getMotors()[4].getCurrentEncoderTicks());
//        telemetry.addLine("Encoder Ticks Right: " + getMotors()[5].getCurrentEncoderTicks());
//        telemetry.addLine("Last Power L: " + getLiftSubsystem().getLiftL().getLastPower());
//        telemetry.addLine("Last Power R: " + getLiftSubsystem().getLiftR().getLastPower());
//        telemetry.addLine("Position L: " + getLiftSubsystem().getLiftL().getPosition());
//        telemetry.addLine("Position R: " + getLiftSubsystem().getLiftR().getPosition());
        updateTelemetry(telemetry);
    }
}
