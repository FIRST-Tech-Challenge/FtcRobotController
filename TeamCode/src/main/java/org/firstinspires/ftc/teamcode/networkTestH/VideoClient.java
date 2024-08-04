import java.awt.image.BufferedImage;
import java.io.*;
import java.net.Socket;
import javax.imageio.ImageIO;

public class VideoClient {
    private static final String SERVER_ADDRESS = "192.168.43.1"; // Replace with your server IP
    private static final int SERVER_PORT = 25565;
    private static boolean running = true;
    private static final String OUTPUT_DIRECTORY = "frames";

    public static void main(String[] args) {
        // Ensure the output directory exists
        File outputDir = new File(OUTPUT_DIRECTORY);
        if (!outputDir.exists()) {
            outputDir.mkdirs();
        }

        while (running) {
            try (Socket socket = new Socket(SERVER_ADDRESS, SERVER_PORT);
                 InputStream is = socket.getInputStream();
                 OutputStream os = socket.getOutputStream();
                 DataOutputStream dos = new DataOutputStream(os);
                 DataInputStream dis = new DataInputStream(is)) {

                int frameCount = 0;

                while (running) {
                    try {
                        // Read the length of the image data
                        int imageSize = dis.readInt();

                        // Check for the special end-of-stream flag
                        if (imageSize == -1) {
                            running = false;
                            break;
                        }

                        byte[] imageData = new byte[imageSize];

                        // Read the image data fully
                        dis.readFully(imageData);

                        // Convert byte array to BufferedImage
                        BufferedImage img = ImageIO.read(new ByteArrayInputStream(imageData));
                        // Save the image to disk
                        File outputFile = new File(outputDir, "frame_" + frameCount + ".jpg");
                        ImageIO.write(img, "jpg", outputFile);

                        System.out.println("Saved frame " + frameCount + " to " + outputFile.getAbsolutePath() + " at " + System.currentTimeMillis());
                        frameCount++;

                        // Send info that the image is being processed
                        dos.writeUTF("pildi protsessimine");
                        dos.flush();

                        // loogika osa pildi protsessimiseks
                        processImage(img);

                        // Send info to the robot to request a new frame
                        dos.writeUTF("ready for next frame");
                        dos.flush();

                    } catch (EOFException e) {
                        System.err.println("EOFException: " + e.getMessage());
                        try {
                            Thread.sleep(1000);
                        } catch (InterruptedException ie) {
                            Thread.currentThread().interrupt();
                            ie.printStackTrace();
                        }
                        break;  // Break the loop to retry connection
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
                try {
                    Thread.sleep(1000);  // Wait before retrying connection
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();  // Restore interrupted status
                    ie.printStackTrace();
                }
            }
        }
    }

    // Simulate some image processing logic (replace with your actual logic)
    private static void processImage(BufferedImage img) {
        // Add your image processing logic here
        // For demonstration, we'll just simulate a delay
        try {
            Thread.sleep(10);  // Simulate a 500 ms processing delay
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();  // Restore interrupted status
            e.printStackTrace();
        }
    }
}
