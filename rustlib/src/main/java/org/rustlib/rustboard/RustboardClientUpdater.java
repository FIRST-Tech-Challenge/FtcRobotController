package org.rustlib.rustboard;

import java.util.Map;
import java.util.Objects;

import javax.json.Json;
import javax.json.JsonArrayBuilder;
import javax.json.JsonObjectBuilder;

public class RustboardClientUpdater extends Thread {
    private static long sleepTime = 20;

    public static void setUpdateFrequency(int frequency) {
        sleepTime = 1000 / frequency;
    }

    @Override
    public void run() {
        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        JsonObjectBuilder messageBuilder = Json.createObjectBuilder();
        messageBuilder.add("action", "update nodes");
        Map<String, RustboardNode> nodeMap = Rustboard.getActiveRustboard().getNodeList();
        JsonArrayBuilder nodes = Json.createArrayBuilder();
        for (String key : nodeMap.keySet()) {
            RustboardNode node = Objects.requireNonNull(nodeMap.get(key));
            nodes.add(node.getJsonBuilder());
            nodeMap.remove(key);
        }
        messageBuilder.add("nodes", nodes);
        RustboardServer.messageActiveRustboard(messageBuilder.build());
    }
}
