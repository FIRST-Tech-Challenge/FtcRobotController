package org.firstinspires.ftc.teamcode.configuration;

import com.pedropathing.geometry.Pose;

public class Settings {
	public static class Rollers {
		public static double INTAKE_SPEED = 0.4;
		public static double INTAKE_STOP = 0;
		public static double INTAKE_OUT = -1;
		public static double MIDDLE_IN = 1;
		public static double MIDDLE_STOP = 0;
		public static double MIDDLE_OUT = -1;

		public static double END_IN = 0.8;
		public static double END_STOP = 0;
		public static double END_OUT = -0.8;
	}

	public static class Hardware {
		public static String TURRET = "turret";
		public static String PINPOINT = "pinpoint";
		public static String LAUNCHER_LEFT = "outtakeleft";
		public static String LAUNCHER_RIGHT = "outtakeright";
		public static String INTAKE = "intake";
		public static String MIDDLE = "middleWheels";
		public static String END = "finalWheels";
	}

	public static class Launcher {
		public static double CLOSE_VELOCITY = 2650;
		public static double FAR_VELOCITY = 3600;
		public static double MAX_SPEED_ERROR = 50;
		public static double TICKS_PER_REVOLUTION = 28;
	}

	public static class Turret {
		public static double SPEED = 0.2;
		public static int MIN_TICKS = -500;   // Tune these
		public static int MAX_TICKS = 500;
	}

	public static class Auto {
		public static double ROLL_TIME = 2.0; // Time in seconds to run rollers
	}

	public static class Positions {
		public static Pose STORED_POSE = new Pose(72, 72, Math.toRadians(90));
		public static Sides SIDE = Sides.BLUE;

		public static class Starting {
			public static Pose close = new Pose(24, 127, Math.toRadians(318));
			public static Pose far = new Pose(60, 7.5, Math.toRadians(90));
		}

		public static class Endgame {
			public static Pose PARK = new Pose(103.5, 34.5, Math.toRadians(90));
		}

		public static class Move {
			public static Pose move = new Pose(35, 7.5, Math.toRadians(90));
		}

		public static class human {
			public static Pose HUMAN = new Pose(130, 7.5, Math.toRadians(90));
		}

		public static class Launch {
			public static Pose close = new Pose(49, 99, Math.toRadians(319));
			public static Pose far = new Pose(59.5, 120, Math.toRadians(355));
			public static Pose far2 = new Pose(55.5, 8, Math.toRadians(280));
		}

		public static class Preset2 {
			public static Pose PREP = new Pose(45, 60, Math.toRadians(180));
			public static Pose END = new Pose(5, 60, Math.toRadians(180));
		}

		public static class Preset3 {
			public static Pose PREP = new Pose(45, 84, Math.toRadians(180));
			public static Pose END = new Pose(12, 84, Math.toRadians(180));
		}

		public static class Park {
			public static Pose close = new Pose(56, 110, 0); // TODO: Set actual park position
		}
	}
}