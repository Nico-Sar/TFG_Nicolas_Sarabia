package com.protocols.magneticSwarmRec.logic;

import com.api.API;
import com.api.communications.HighlevelCommLink;
import com.api.copter.Copter;
import com.protocols.magneticSwarmRec.pojo.Message;
import com.protocols.magneticSwarmRec.pojo.UAVStateEntry;
import es.upv.grc.mapper.Location3DUTM;
import org.javatuples.Pair;
import org.json.JSONObject;
import com.protocols.magneticSwarmRec.pojo.Vector;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class Communication extends Thread {

    private static final int BEACONING_TIME = 100; // ms fijo

    private final int numUAV;
    private final Copter copter;
    private final HighlevelCommLink commLink;
    private final Map<Integer, UAVStateEntry> locations;
    private final Map<Integer, Pair<Location3DUTM, Vector>> projectedRepulsions;

    private boolean running;
    private boolean isHovering = false;

    public Communication(int numUAV) {
        this.numUAV = numUAV;
        this.copter = API.getCopter(numUAV);
        this.commLink = new HighlevelCommLink(numUAV, 1, 1);
        this.locations = new ConcurrentHashMap<>();
        this.projectedRepulsions = new ConcurrentHashMap<>();
        this.running = true;
    }

    public void setHovering(boolean hovering) {
        this.isHovering = hovering;
    }

    @Override
    public void run() {
        while (running) {
            long start = System.currentTimeMillis();

            try {
                commLink.sendJSON(Message.location(numUAV, getCopterLocation(), getHeading(), isHovering ? "HOVERING" : "MOVING"));
            } catch (Exception e) {
                e.printStackTrace();
            }

            long timeDif = System.currentTimeMillis() - start;
            try {
                JSONObject msg = commLink.receiveMessage(Message.location(numUAV));
                long currentTime = System.currentTimeMillis();

                while (msg != null && timeDif < BEACONING_TIME) {
                    int senderId = (Integer) msg.get(HighlevelCommLink.Keywords.SENDERID);
                    Pair<Location3DUTM, Double> data = Message.processLocationWithHeading(msg);
                    String state = Message.getState(msg);
                    locations.put(senderId, new UAVStateEntry(currentTime, data, state));

                    msg = commLink.receiveMessage(Message.location(numUAV));
                    timeDif = System.currentTimeMillis() - start;
                }
            } catch (Exception e) {
                e.printStackTrace();
            }

            if (timeDif < BEACONING_TIME) {
                API.getArduSim().sleep(BEACONING_TIME - timeDif);
            }
        }
    }

    public List<Pair<Location3DUTM, Double>> getObstaclesWithHeading() {
        List<Pair<Location3DUTM, Double>> obstacles = new ArrayList<>();
        long now = System.currentTimeMillis();
        Set<Integer> expired = new HashSet<>();

        for (Map.Entry<Integer, UAVStateEntry> entry : locations.entrySet()) {
            int senderId = entry.getKey();
            if (senderId == numUAV) continue;

            if (now - entry.getValue().timestamp > 5000) {
                expired.add(senderId);
            } else {
                String state = entry.getValue().state;
                if (state.equals("MOVING") || state.equals("HOVERING")) {
                    obstacles.add(entry.getValue().positionAndHeading);
                }
            }
        }

        for (Integer id : expired) {
            locations.remove(id);
        }

        return obstacles;
    }

    public boolean allUAVsHovering() {
        int total = API.getArduSim().getNumUAVs();
        int count = 1; // este UAV

        for (Map.Entry<Integer, UAVStateEntry> entry : locations.entrySet()) {
            if (entry.getKey() == numUAV) continue;
            if ("HOVERING".equals(entry.getValue().state)) {
                count++;
            }
        }
        return count == total;
    }

    public void stopCommunication() {
        running = false;
    }

    private Location3DUTM getCopterLocation() {
        return new Location3DUTM(copter.getLocationUTM(), 0);
    }

    public boolean isUAVHovering(Location3DUTM location) {
        for (Map.Entry<Integer, UAVStateEntry> entry : locations.entrySet()) {
            Location3DUTM loc = entry.getValue().positionAndHeading.getValue0();
            if (loc != null && loc.distance3D(location) < 1.0) {
                return "HOVERING".equals(entry.getValue().state);
            }
        }
        return false;
    }

    private static Location3DUTM sharedCenter;

    public void broadcastCenter(Location3DUTM center) {
        sharedCenter = center;
    }

    public Location3DUTM waitForCenter() {
        while (sharedCenter == null) {
            API.getArduSim().sleep(50);
        }
        return sharedCenter;
    }

    private double getHeading() {
        return Math.toRadians(copter.getHeading());
    }

    // 🚀 NUEVO: Actualizar campo de repulsión proyectado
    public void updateProjectedRepulsion(int uavId, Location3DUTM location, Vector escapeVector) {
        projectedRepulsions.put(uavId, new Pair<>(location, escapeVector));
    }

    // 🚀 NUEVO: Obtener campos de repulsión proyectados por otros UAVs
    public List<Pair<Location3DUTM, Vector>> getProjectedRepulsions(int selfId) {
        List<Pair<Location3DUTM, Vector>> result = new ArrayList<>();
        for (Map.Entry<Integer, Pair<Location3DUTM, Vector>> entry : projectedRepulsions.entrySet()) {
            if (entry.getKey() != selfId) {
                result.add(entry.getValue());
            }
        }
        return result;
    }
}