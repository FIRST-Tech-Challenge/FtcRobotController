package org.firstinspires.ftc.teamcode.tools;

import android.content.Context;
import com.qualcomm.robotcore.util.RobotLog;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import org.firstinspires.ftc.ftccommon.external.OnCreate;

/**
 * Opens the Limelight's own web page through the Control Hub, so nobody has to unplug the
 * Limelight to change a pipeline.
 *
 * <p>The Limelight hangs off the hub's USB port with its own little network: it is always
 * {@code 172.29.0.1}, and only the hub can see it. A laptop on the robot's Wi-Fi sees the
 * hub at {@code 192.168.43.1}. This class makes the hub pass traffic through: anything
 * that arrives at the hub on ports 5800 to 5807 is copied to the Limelight on the same
 * port, and the answers are copied back. So, on a laptop joined to the robot's Wi-Fi:
 *
 * <pre>
 *     http://192.168.43.1:5801   the Limelight's web page (pipelines, ID filter, live view)
 *     http://192.168.43.1:5800   its video stream
 * </pre>
 *
 * <p>It is the same idea as WPILib's PortForwarder that the Limelight docs show for FRC.
 * It starts once, when the Robot Controller app starts, and runs in the background.
 *
 * <p>This is a mentor's tool: sockets and threads, not the robot loop. Nothing in the
 * robot code depends on it. If a port can't be opened the problem is written to the
 * Robot Controller log and the rest of the app carries on.
 */
public final class LimelightWebProxy {
    private static final String TAG = "LimelightWebProxy";
    /** Where the Limelight always is, on the hub's USB network. */
    static final String LIMELIGHT_ADDRESS = "172.29.0.1";
    /** The Limelight's ports: 5800 video, 5801 web page, 5802 to 5807 what the page talks to. */
    static final int FIRST_PORT = 5800;
    static final int LAST_PORT = 5807;

    private static boolean sStarted = false;

    private LimelightWebProxy() {
    }

    /** The Robot Controller app calls this once when it starts. */
    @OnCreate
    public static void start(Context context) {
        if (sStarted) {
            return;
        }
        sStarted = true;
        for (int port = FIRST_PORT; port <= LAST_PORT; port++) {
            new Relay(port, LIMELIGHT_ADDRESS, port).start();
        }
        RobotLog.ii(TAG, "Limelight web page is at http://192.168.43.1:5801 while on the robot's Wi-Fi");
    }

    /**
     * One listening port. Each connection that arrives is paired with a new connection to
     * the Limelight, and bytes are copied both ways until one side hangs up.
     */
    static final class Relay extends Thread {
        private final int mListenPort;
        private final String mToHost;
        private final int mToPort;
        private ServerSocket mServer;

        Relay(int listenPort, String toHost, int toPort) {
            super("LimelightWebProxy-" + listenPort);
            mListenPort = listenPort;
            mToHost = toHost;
            mToPort = toPort;
            setDaemon(true);
        }

        /** Open the listening port. Separate from run() so a test can learn the port it got. */
        synchronized int open() throws IOException {
            if (mServer == null) {
                mServer = new ServerSocket(mListenPort);
            }
            return mServer.getLocalPort();
        }

        @Override
        public void run() {
            try {
                open();
            } catch (IOException e) {
                RobotLog.ww(TAG, "could not open port " + mListenPort + ": " + e.getMessage());
                return;
            }
            while (true) {
                try {
                    Socket fromLaptop = mServer.accept();
                    Socket toLimelight = new Socket(mToHost, mToPort);
                    new Copier(fromLaptop.getInputStream(), toLimelight.getOutputStream(), toLimelight).start();
                    new Copier(toLimelight.getInputStream(), fromLaptop.getOutputStream(), fromLaptop).start();
                } catch (IOException e) {
                    // The Limelight is unplugged, or the laptop gave up. Wait for the next one.
                    RobotLog.vv(TAG, "port " + mListenPort + ": " + e.getMessage());
                }
            }
        }
    }

    /** Copies bytes one way until the stream ends, then closes the other side's socket. */
    static final class Copier extends Thread {
        private final InputStream mFrom;
        private final OutputStream mTo;
        private final Socket mCloseWhenDone;

        Copier(InputStream from, OutputStream to, Socket closeWhenDone) {
            super("LimelightWebProxy-copy");
            mFrom = from;
            mTo = to;
            mCloseWhenDone = closeWhenDone;
            setDaemon(true);
        }

        @Override
        public void run() {
            byte[] buffer = new byte[8192];
            try {
                while (true) {
                    int count = mFrom.read(buffer);
                    if (count < 0) {
                        break;
                    }
                    mTo.write(buffer, 0, count);
                    mTo.flush();
                }
            } catch (IOException e) {
                // One side hung up. That is how every web connection ends.
            }
            try {
                mCloseWhenDone.close();
            } catch (IOException e) {
                // Already closed.
            }
        }
    }
}
