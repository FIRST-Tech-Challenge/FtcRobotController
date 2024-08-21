package org.rustlib.rustboard;

import static org.rustlib.rustboard.MessageActions.MESSAGE_ACTION_KEY;
import static org.rustlib.rustboard.MessageActions.UPDATE_NODES;

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
    public synchronized void run() {
        try {
            JsonObjectBuilder messageBuilder = Json.createObjectBuilder();
            messageBuilder.add(MESSAGE_ACTION_KEY, UPDATE_NODES);
            JsonArrayBuilder nodes = Json.createArrayBuilder();
            Map<String, RustboardNode> nodeMap = new ConcurrentHashMap<>(toUpdate); // Copy the map to avoid concurrency issues in iteration
            Set<String> keySet = nodeMap.keySet();
            if (keySet.size() == 0) {
                return;
            }
            for (String key : keySet) {
                RustboardNode node = Objects.requireNonNull(nodeMap.get(key));
                nodes.add(node.getJsonBuilder());
                toUpdate.remove(key);
            }
            messageBuilder.add(RustboardNode.NODE_ARRAY_KEY, nodes);
            RustboardServer.messageActiveRustboard(messageBuilder.build());
        } catch (Exception e) {
            RustboardServer.warnClientConsoles(e);
        }
    }
}
