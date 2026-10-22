package org.firstinspires.ftc.teamcode.dcs15815.BuzzBot;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dcs15815.DefenderFramework.DefenderBot.DefenderBot;
import org.firstinspires.ftc.teamcode.dcs15815.DefenderFramework.DefenderUtilities.DefenderPIDController;

import java.util.function.BooleanSupplier;

public class BuzzBot extends DefenderBot {

	private boolean useDebugging = false;
	public BooleanSupplier abortOpMode = () -> false;


	public BuzzBot(HardwareMap hm, Class configClass, Telemetry t) {
		super(hm, configClass, t);

//		drivetrain = addSystem(new DecodeMecanumDrivetrain(hm, this));

//		navigation = addSystem(new DecodeNavigation(hm, this));

	}

	public void setUseDebugging(boolean b) {
		useDebugging = b;
	}



}