package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;
import android.content.Context;

import com.acmerobotics.roadrunner.Pose2d;
import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectWriter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Objects;

import fi.iki.elonen.NanoHTTPD;

public final class LogFiles {
    private static final File ROOT =
            new File(AppUtil.ROOT_FOLDER + "/RoadRunner/logs/");

    public static LogFile log = new LogFile("uninitialized");

    public static class LogFile {
        public String version = "quickstart2 v0";

        public String opModeName;
        public long msInit = System.currentTimeMillis();
        public long nsInit = System.nanoTime();
        public long nsStart, nsStop;

        public String driveClassName = TuningOpModes.DRIVE_CLASS.getCanonicalName();

        public double mecInPerTick = MecanumDrive.IN_PER_TICK;
        public double mecLateralInPerTick = MecanumDrive.LATERAL_IN_PER_TICK;
        public double mecTrackWidthTicks = MecanumDrive.TRACK_WIDTH_TICKS;
        public double mecKS = MecanumDrive.kS;
        public double mecKV = MecanumDrive.kV;
        public double mecKA = MecanumDrive.kA;
        public double mecMaxWheelVel = MecanumDrive.MAX_WHEEL_VEL;
        public double mecMinProfileAccel = MecanumDrive.MIN_PROFILE_ACCEL;
        public double mecMaxProfileAccel = MecanumDrive.MAX_PROFILE_ACCEL;
        public double mecMaxAngVel = MecanumDrive.MAX_ANG_VEL;
        public double mecMaxAngAccel = MecanumDrive.MAX_ANG_ACCEL;
        public double mecAxialGain = MecanumDrive.AXIAL_GAIN;
        public double mecLateralGain = MecanumDrive.LATERAL_GAIN;
        public double mecHeadingGain = MecanumDrive.HEADING_GAIN;
        public double mecAxialVelGain = MecanumDrive.AXIAL_VEL_GAIN;
        public double mecLateralVelGain = MecanumDrive.LATERAL_VEL_GAIN;
        public double mecHeadingVelGain = MecanumDrive.HEADING_VEL_GAIN;

        public double tankInPerTick = TankDrive.IN_PER_TICK;
        public double tankTrackWidthTicks = TankDrive.TRACK_WIDTH_TICKS;
        public double tankKS = TankDrive.kS;
        public double tankKV = TankDrive.kV;
        public double tankKA = TankDrive.kA;
        public double tankMaxWheelVel = TankDrive.MAX_WHEEL_VEL;
        public double tankMinProfileAccel = TankDrive.MIN_PROFILE_ACCEL;
        public double tankMaxProfileAccel = TankDrive.MAX_PROFILE_ACCEL;
        public double tankMaxAngVel = TankDrive.MAX_ANG_VEL;
        public double tankMaxAngAccel = TankDrive.MAX_ANG_ACCEL;
        public double tankRamseteZeta = TankDrive.RAMSETE_ZETA;
        public double tankRamseteBbar = TankDrive.RAMSETE_BBAR;
        public double tankTurnGain = TankDrive.TURN_GAIN;
        public double tankTurnVelGain = TankDrive.TURN_VEL_GAIN;

        public double threePar0YTicks = ThreeDeadWheelLocalizer.PAR0_Y_TICKS;
        public double threePar1YTicks = ThreeDeadWheelLocalizer.PAR1_Y_TICKS;
        public double threePerpXTicks = ThreeDeadWheelLocalizer.PERP_X_TICKS;

        public double twoParYTicks = TwoDeadWheelLocalizer.PAR_Y_TICKS;
        public double twoPerpXTicks = TwoDeadWheelLocalizer.PERP_X_TICKS;

        public List<Long> nsTimes = new ArrayList<>();

        public List<Double> targetXs = new ArrayList<>();
        public List<Double> targetYs = new ArrayList<>();
        public List<Double> targetHeadings = new ArrayList<>();

        public List<Double> xs = new ArrayList<>();
        public List<Double> ys = new ArrayList<>();
        public List<Double> headings = new ArrayList<>();

        public LogFile(String opModeName) {
            this.opModeName = opModeName;
        }
    }

    public static void recordTargetPose(Pose2d targetPose) {
        log.targetXs.add(targetPose.position.x);
        log.targetYs.add(targetPose.position.y);
        log.targetHeadings.add(targetPose.heading.log());
    }

    public static void recordPose(Pose2d pose) {
        // arbitrarily add time here
        log.nsTimes.add(System.nanoTime());

        log.xs.add(pose.position.x);
        log.ys.add(pose.position.y);
        log.headings.add(pose.heading.log());
    }

    private static final OpModeManagerNotifier.Notifications notifHandler = new OpModeManagerNotifier.Notifications() {
        @SuppressLint("SimpleDateFormat")
        final DateFormat dateFormat = new SimpleDateFormat("yyyy_MM_dd__HH_mm_ss_SSS");

        final ObjectWriter jsonWriter = new ObjectMapper(new JsonFactory())
                .writerWithDefaultPrettyPrinter();

        @Override
        public void onOpModePreInit(OpMode opMode) {
            log = new LogFile(opMode.getClass().getCanonicalName());

            // clean up old files
            File[] fs = Objects.requireNonNull(ROOT.listFiles());
            Arrays.sort(fs, (a, b) -> Long.compare(a.lastModified(), b.lastModified()));
            long totalSizeBytes = 0;
            for (File f : fs) {
                totalSizeBytes += f.length();
            }

            int i = 0;
            while (i < fs.length && totalSizeBytes >= 8 * 1000 * 1000) {
                totalSizeBytes -= fs[i].length();
                if (!fs[i].delete()) {
                    RobotLog.setGlobalErrorMsg("Unable to delete file " + fs[i].getAbsolutePath());
                }
                ++i;
            }
        }

        @Override
        public void onOpModePreStart(OpMode opMode) {
            log.nsStart = System.nanoTime();
        }

        @Override
        public void onOpModePostStop(OpMode opMode) {
            log.nsStop = System.nanoTime();

            if (!(opMode instanceof OpModeManagerImpl.DefaultOpMode)) {
                String filename = dateFormat.format(new Date(log.msInit)) + "__" + opMode.getClass().getSimpleName() + ".json";
                File file = new File(ROOT, filename);
                try {
                    jsonWriter.writeValue(file, log);
                } catch (IOException e) {
                    RobotLog.setGlobalErrorMsg(new RuntimeException(e),
                            "Unable to write data to " + file.getAbsolutePath());
                }
            }
        }
    };

    @WebHandlerRegistrar
    public static void registerRoutes(Context context, WebHandlerManager manager) {
        //noinspection ResultOfMethodCallIgnored
        ROOT.mkdirs();

        // op mode manager only stores a weak reference, so we need to keep notifHandler alive ourselves
        // don't use @OnCreateEventLoop because it's unreliable
        OpModeManagerImpl.getOpModeManagerOfActivity(
                AppUtil.getInstance().getActivity()
        ).registerListener(notifHandler);

        manager.register("/logs", session -> {
            final StringBuilder sb = new StringBuilder();
            sb.append("<!doctype html><html><head><title>Logs</title></head><body><ul>");
            File[] fs = Objects.requireNonNull(ROOT.listFiles());
            Arrays.sort(fs, (a, b) -> Long.compare(b.lastModified(), a.lastModified()));
            for (File f : fs) {
                sb.append("<li><a href=\"/logs/download?file=");
                sb.append(f.getName());
                sb.append("\" download=\"");
                sb.append(f.getName());
                sb.append("\">");
                sb.append(f.getName());
                sb.append("</a></li>");
            }
            sb.append("</ul></body></html>");
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK,
                    NanoHTTPD.MIME_HTML, sb.toString());
        });

        manager.register("/logs/download", session -> {
            final String[] pairs = session.getQueryParameterString().split("&");
            if (pairs.length != 1) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST,
                        NanoHTTPD.MIME_PLAINTEXT, "expected one query parameter, got " + pairs.length);
            }

            final String[] parts = pairs[0].split("=");
            if (!parts[0].equals("file")) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST,
                        NanoHTTPD.MIME_PLAINTEXT, "expected file query parameter, got " + parts[0]);
            }

            File f = new File(ROOT, parts[1]);
            if (!f.exists()) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                        NanoHTTPD.MIME_PLAINTEXT, "file " + f + " doesn't exist");
            }

            return NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                    "application/json", new FileInputStream(f));
        });
    }
}
