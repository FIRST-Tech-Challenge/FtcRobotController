package org.firstinspires.ftc.teamcode.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * The relay with a pretend Limelight on this laptop: a server that answers every request
 * with "limelight says: " and the request echoed back.
 */
@DisplayName("LimelightWebProxy")
class LimelightWebProxyTest {

    /** A stand-in Limelight on a free local port. Answers one request, then hangs up. */
    private static ServerSocket pretendLimelight() throws Exception {
        ServerSocket server = new ServerSocket(0);
        Thread answerer = new Thread(() -> {
            try (Socket client = server.accept()) {
                byte[] buffer = new byte[256];
                int count = client.getInputStream().read(buffer);
                String request = new String(buffer, 0, count, StandardCharsets.UTF_8);
                OutputStream out = client.getOutputStream();
                out.write(("limelight says: " + request).getBytes(StandardCharsets.UTF_8));
                out.flush();
            } catch (Exception ignored) {
                // the test will fail on the missing answer
            }
        });
        answerer.setDaemon(true);
        answerer.start();
        return server;
    }

    @Test
    @DisplayName("a request to the relay reaches the Limelight, and its answer comes back")
    void bytesGoBothWays() throws Exception {
        ServerSocket limelight = pretendLimelight();
        LimelightWebProxy.Relay relay = new LimelightWebProxy.Relay(0, "127.0.0.1", limelight.getLocalPort());
        int relayPort = relay.open();
        relay.start();

        try (Socket laptop = new Socket("127.0.0.1", relayPort)) {
            laptop.setSoTimeout(3000);
            laptop.getOutputStream().write("GET /pipelines".getBytes(StandardCharsets.UTF_8));
            laptop.getOutputStream().flush();
            InputStream in = laptop.getInputStream();
            byte[] buffer = new byte[256];
            int total = 0;
            int count;
            while ((count = in.read(buffer, total, buffer.length - total)) > 0) {
                total += count;
            }
            assertEquals("limelight says: GET /pipelines", new String(buffer, 0, total, StandardCharsets.UTF_8));
        }
    }

    @Test
    @DisplayName("the eight Limelight ports are 5800 to 5807, at 172.29.0.1")
    void thePortsTheLimelightUses() {
        assertEquals(5800, LimelightWebProxy.FIRST_PORT);
        assertEquals(5807, LimelightWebProxy.LAST_PORT);
        assertEquals("172.29.0.1", LimelightWebProxy.LIMELIGHT_ADDRESS);
    }
}
