package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AutoControl.AutoControl;
import org.firstinspires.ftc.teamcode.Control;


public abstract class Auto extends AutoControl {
   AutoControl autoControl = new AutoControl();
   @Override
   public void loop() {
      autoControl.AutoDrive( 10, 0);
      autoControl.AutoTurn(0);
   }

}
