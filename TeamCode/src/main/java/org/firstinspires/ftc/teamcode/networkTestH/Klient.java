import java.awt.image.BufferedImage;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ByteArrayInputStream;
import java.net.Socket;
import javax.imageio.ImageIO;
import java.io.File;

public class VideoClient {
    private static final String SERVER_ADDRESS = "192.168.43.1"; // Replace with your server IP
    private static final int SERVER_PORT = 25565;

    public static void main(String[] args) {
        try (Socket socket = new Socket(SERVER_ADDRESS, SERVER_PORT);
             InputStream is = socket.getInputStream();
             DataInputStream dis = new DataInputStream(is)) {

            int frameCount = 0;

            while (true) {
                // Read the length of the image data
                int imageSize = dis.readInt();
                byte[] imageData = new byte[imageSize];

                // Read the image data
                dis.readFully(imageData);

                // Convert byte array to BufferedImage
                BufferedImage img = ImageIO.read(new ByteArrayInputStream(imageData));

                // Save the image to disk
                File outputFile = new File("frame_" + frameCount + ".jpg");
                ImageIO.write(img, "jpg", outputFile);

                System.out.println("Saved frame " + frameCount + " to " + outputFile.getAbsolutePath() + " at " + System.currentTimeMillis());
                frameCount++;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
