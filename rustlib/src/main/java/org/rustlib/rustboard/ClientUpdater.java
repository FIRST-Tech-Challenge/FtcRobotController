package org.rustlib.rustboard;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import javax.json.Json;
import javax.json.JsonArrayBuilder;
import javax.json.JsonObjectBuilder;

public class ClientUpdater implements Runnable {
    private final Map<String, RustboardNode> toUpdate = new ConcurrentHashMap<>();

    void updateNode(RustboardNode node) {
        toUpdate.put(node.id, node);
    }

    @Override
    public void run() {
        Rustboard.notifyAllClients("running client updater", NoticeType.POSITIVE);
        JsonObjectBuilder messageBuilder = Json.createObjectBuilder();
        messageBuilder.add("action", "update_nodes");
        JsonArrayBuilder nodes = Json.createArrayBuilder();
        Map<String, RustboardNode> nodeMap = new HashMap<>(toUpdate); // Copy the map to avoid concurrency issues in iteration
        Set<String> keySet = nodeMap.keySet();
        if (keySet.size() == 0) {
            return;
        }
        for (String key : keySet) {
            RustboardNode node = Objects.requireNonNull(nodeMap.get(key));
            nodes.add(node.getJsonBuilder());
            toUpdate.remove(key);
        }
        messageBuilder.add("nodes", nodes);
        RustboardServer.messageActiveRustboard(messageBuilder.build());
    }
}
