package org.rustlib.rustboard;

import static org.rustlib.rustboard.JsonKeys.NODE_ARRAY_KEY;
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
    private static class ClientUpdateRequest {
        private final RustboardNode node;
        private final long timestamp;

        private ClientUpdateRequest(RustboardNode node) {
            this.node = node;
            this.timestamp = System.currentTimeMillis();
        }
    }

    private final Map<String, ClientUpdateRequest> toUpdate = new ConcurrentHashMap<>();

    void updateNode(RustboardNode node) {
        toUpdate.put(node.id, new ClientUpdateRequest(node));
    }

    @Override
    public synchronized void run() {
        try {
            JsonObjectBuilder messageBuilder = Json.createObjectBuilder();
            messageBuilder.add(MESSAGE_ACTION_KEY, UPDATE_NODES);
            JsonArrayBuilder nodes = Json.createArrayBuilder();
            Map<String, ClientUpdateRequest> nodeMap = new ConcurrentHashMap<>(toUpdate); // Copy the map to avoid concurrency issues in iteration
            Set<String> keySet = nodeMap.keySet();
            if (keySet.size() == 0) {
                return;
            }
            for (String key : keySet) {
                ClientUpdateRequest processedRequest = Objects.requireNonNull(nodeMap.get(key));
                RustboardNode node = processedRequest.node;
                nodes.add(node.getJsonBuilder());
                ClientUpdateRequest latestRequest = toUpdate.get(key);
                if (latestRequest != null && latestRequest.timestamp == processedRequest.timestamp) { // Perhaps a new request to update the same node was added after the hashmap was copied and before the request would normally get removed.  If this is true, then don't remove that request!
                    toUpdate.remove(key);
                }
            }
            messageBuilder.add(NODE_ARRAY_KEY, nodes);
            RustboardServer.messageActiveRustboard(messageBuilder.build());
        } catch (Exception e) {
            RustboardServer.warnClientConsoles(e);
        }
    }
}
