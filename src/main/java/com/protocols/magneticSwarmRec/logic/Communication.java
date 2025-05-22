package com.protocols.magneticSwarmRec.logic;

import com.api.API;
import com.api.communications.HighlevelCommLink;
import com.api.copter.Copter;
import com.protocols.magneticSwarmRec.gui.magneticSwarmRecSimProperties;
import com.protocols.magneticSwarmRec.pojo.Message;
import es.upv.grc.mapper.Location3DUTM;
import org.javatuples.Pair;
import org.json.JSONObject;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class Communication extends Thread {

    private final int numUAV;
    private final Copter copter;
    private final HighlevelCommLink commLink;
    private final Map<Integer, Pair<Long, Pair<Location3DUTM, Double>>> locations; // heading incluido
    private boolean running;

    public Communication(int numUAV) {
        this.numUAV = numUAV;
        this.copter = API.getCopter(numUAV);
        this.commLink = new HighlevelCommLink(numUAV, 1, 1);
        this.locations = new ConcurrentHashMap<>();
        this.running = true;
    }

    @Override
    public void run() {
        while (running) {
            long start = System.currentTimeMillis();

            try {
                commLink.sendJSON(Message.location(numUAV, getCopterLocation(), getHeading()));
            } catch (Exception e) {
                e.printStackTrace();
            }

            long timeDif = System.currentTimeMillis() - start;
            try {
                JSONObject msg = commLink.receiveMessage(Message.location(numUAV));
                long currentTime = System.currentTimeMillis();

                while (msg != null && timeDif < magneticSwarmRecSimProperties.beaconingTime) {
                    int senderId = (Integer) msg.get(HighlevelCommLink.Keywords.SENDERID);
                    Pair<Location3DUTM, Double> data = Message.processLocationWithHeading(msg);
                    locations.put(senderId, new Pair<>(currentTime, data));

                    msg = commLink.receiveMessage(Message.location(numUAV));
                    timeDif = System.currentTimeMillis() - start;
                }
            } catch (Exception e) {
                e.printStackTrace();
            }

            if (timeDif < magneticSwarmRecSimProperties.beaconingTime) {
                API.getArduSim().sleep(magneticSwarmRecSimProperties.beaconingTime - timeDif);
            }
        }
    }

    public List<Pair<Location3DUTM, Double>> getObstaclesWithHeading() {
        List<Pair<Location3DUTM, Double>> obstacles = new ArrayList<>();
        long now = System.currentTimeMillis();
        Set<Integer> expired = new HashSet<>();

        for (Map.Entry<Integer, Pair<Long, Pair<Location3DUTM, Double>>> entry : locations.entrySet()) {
            int senderId = entry.getKey();
            if (senderId == numUAV) continue;

            if (now - entry.getValue().getValue0() > 5000) {
                expired.add(senderId);
            } else {
                obstacles.add(entry.getValue().getValue1());
            }
        }


        for (Integer id : expired) {
            locations.remove(id);
        }

        return obstacles;
    }

    public void stopCommunication() {
        running = false;
    }

    private Location3DUTM getCopterLocation() {
        return new Location3DUTM(copter.getLocationUTM(), 0);
    }

    private double getHeading() {
        return Math.toRadians(copter.getHeading()); // conversión a radianes
    }
}
