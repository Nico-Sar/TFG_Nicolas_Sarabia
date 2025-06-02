package com.protocols.magneticSwarmRec.logic;

import com.api.API;
import com.api.copter.Copter;
import com.api.copter.TakeOff;
import com.api.copter.TakeOffListener;
import com.api.swarm.formations.Formation;
import com.api.swarm.formations.FormationFactory;
import com.protocols.magneticSwarmRec.gui.magneticSwarmRecSimProperties;
import com.protocols.magneticSwarmRec.pojo.HungarianAlgorithm;
import com.protocols.magneticSwarmRec.pojo.Vector;
import com.uavController.UAVParam;
import es.upv.grc.mapper.Location3DGeo;
import es.upv.grc.mapper.Location3DUTM;
import org.javatuples.Pair;

import java.util.*;
import java.util.concurrent.CyclicBarrier;

public class magneticSwarmAvoidance extends Thread {
    private Queue<Location3DUTM> waypoints;
    private final Copter copter;
    private final int numUAV, numUAVs;
    private final Communication communication;
    private final double maxspeed;
    private final DrawVectors drawVectors;
    private static CyclicBarrier barrier, landingBarrier;
    private int framesNearTarget = 0, attempts = 0;
    private final int FRAMES_TO_CONFIRM = 30, maxAttempts = 100;
    private final double threshold = 2.0;
    private UAVState state = UAVState.MOVING;

    private enum UAVState { MOVING, HOVERING, LANDING, IDLE }

    public magneticSwarmAvoidance(int numUAV) {
        this.numUAV = numUAV;
        this.numUAVs = API.getArduSim().getNumUAVs();
        this.copter = API.getCopter(numUAV);
        this.drawVectors = new DrawVectors(copter);
        this.communication = new Communication(numUAV);
        this.maxspeed = copter.getPlannedSpeed();
        this.waypoints = new LinkedList<>();
        synchronized (magneticSwarmAvoidance.class) {
            if (barrier == null) {
                barrier = new CyclicBarrier(numUAVs);
                landingBarrier = new CyclicBarrier(numUAVs);
            }
        }
    }

    @Override
    public void run() {
        takeoff();
        communication.start();
        switchToFlyingFormation();
        while (true) {
            switch (state) {
                case MOVING:
                    if (waypoints.isEmpty()) {
                        state = UAVState.HOVERING;
                        break;
                    }
                    Location3DUTM destino = waypoints.peek();
                    double dist = getCopterLocation().distance3D(destino);
                    Vector total = Vector.add(getAttractionVector(), (dist < 2.5) ? new Vector(0, 0) : calculateHybridRepulsion());
                    sendVelocityCommand(total);
                    if (dist < threshold || (dist < 3.5 && attempts > 40)) {
                        waypoints.poll();
                        attempts = 0;
                    } else attempts++;
                    break;
                case HOVERING:
                    communication.setHovering(true);
                    if (communication.allUAVsHovering()) {
                        try {
                            landingBarrier.await();
                            state = UAVState.LANDING;
                        } catch (Exception ignored) {}
                    } else {
                        Vector correction = new Vector(getCopterLocation(), getLastTarget());
                        correction.normalize();
                        correction.scalarProduct(maxspeed / 4);
                        moveUAV(correction);
                    }
                    break;
                case LANDING:
                    communication.stopCommunication();
                    copter.land();
                    return;
            }
            API.getArduSim().sleep(100);
        }
    }

    private void takeoff() {
        TakeOff takeOff = copter.takeOff(magneticSwarmRecSimProperties.altitude, new TakeOffListener() {
            public void onCompleteActionPerformed() {}
            public void onFailure() {}
        });
        takeOff.start();
        try { takeOff.join(); barrier.await(); } catch (Exception ignored) {}
    }

    private void switchToFlyingFormation() {
        System.out.printf("UAV %d - Iniciando switchToFlyingFormation()\n", numUAV);

        Formation target = FormationFactory.newFormation(
                Formation.Layout.valueOf(magneticSwarmRecSimProperties.flyingFormation.toUpperCase())
        );

        target.init(numUAVs, magneticSwarmRecSimProperties.flyingDistance);
        double alt = magneticSwarmRecSimProperties.altitude;

        double latSum = 0.0, lonSum = 0.0;
        for (int i = 0; i < numUAVs; i++) {
            latSum += API.getCopter(i).getLocation().getLatitude();
            lonSum += API.getCopter(i).getLocation().getLongitude();
        }

        double centerLat = latSum / numUAVs;
        double centerLon = lonSum / numUAVs;
        Location3DUTM centerUTM = Location3DGeo.getUTM(centerLat, centerLon, alt);

        // Debug: imprimir posiciones generadas
        System.out.println("=== POSICIONES DE LA FORMACIÓN ===");
        for (int i = 0; i < numUAVs; i++) {
            Location3DUTM p = target.get3DUTMLocation(centerUTM, i);
            System.out.printf("[%d] -> (%.2f, %.2f, %.2f)\n", i, p.x, p.y, p.z);
        }

        Location3DUTM[] formationPositions = new Location3DUTM[numUAVs];
        Location3DUTM[] currentPositions = new Location3DUTM[numUAVs];
        for (int i = 0; i < numUAVs; i++) {
            formationPositions[i] = target.get3DUTMLocation(centerUTM, i);
            currentPositions[i] = new Location3DUTM(API.getCopter(i).getLocationUTM(), alt);
        }

        double[][] costMatrix = new double[numUAVs][numUAVs];
        for (int i = 0; i < numUAVs; i++) {
            for (int j = 0; j < numUAVs; j++) {
                costMatrix[i][j] = currentPositions[i].distance3D(formationPositions[j]);
            }
        }

        int[] assignment = new HungarianAlgorithm(costMatrix).execute();

        System.out.println("=== ASIGNACIÓN HÚNGARA ===");
        for (int i = 0; i < numUAVs; i++) {
            int assigned = assignment[i];
            Location3DUTM start = currentPositions[i];
            Location3DUTM goal = formationPositions[assigned];
            System.out.printf("UAV %d -> Posición %d | (%.1f,%.1f) -> (%.1f,%.1f) | Dist=%.2f\n",
                    i, assigned, start.x, start.y, goal.x, goal.y, start.distance3D(goal));


        }

        int assignedIndex = (numUAV < assignment.length) ? assignment[numUAV] : 0;
        Location3DUTM targetPos = formationPositions[assignedIndex];
        Location3DUTM current = getCopterLocation();

        double midX = current.x + 0.7 * (targetPos.x - current.x);
        double midY = current.y + 0.7 * (targetPos.y - current.y);
        Location3DUTM intermediate = new Location3DUTM(midX, midY, alt);

        waypoints.clear();
        waypoints.add(intermediate);
        waypoints.add(targetPos);
    }




    private Vector getAttractionVector() {
        Vector attraction = new Vector(getCopterLocation(), waypoints.peek());
        attraction.normalize();
        double dist = getCopterLocation().distance3D(waypoints.peek());
        double speed = dist > 50 ? maxspeed :
                dist > 30 ? maxspeed * 0.8 :
                        dist > 10 ? maxspeed * 0.6 :
                                dist > 2 ? maxspeed * 0.4 : maxspeed * 0.2;
        if (dist < 10) speed *= 1.5;
        if (dist < 4.0 && attempts > 40) speed *= 0.6;
        attraction.scalarProduct(speed);
        return attraction;
    }

    private Vector calculateHybridRepulsion() {
        Vector total = new Vector();
        List<Pair<Location3DUTM, Double>> obstacles = communication.getObstaclesWithHeading();
        Vector headingVec = new Vector(Math.cos(getHeading()), Math.sin(getHeading()));
        double cutoff = magneticSwarmRecSimProperties.frd;
        double mu = magneticSwarmRecSimProperties.dirFactor;
        double dirRatio = magneticSwarmRecSimProperties.dirRatio;

        for (Pair<Location3DUTM, Double> obs : obstacles) {
            Location3DUTM obstacle = obs.getValue0();
            double dist = getCopterLocation().distance3D(obstacle);
            if (dist < cutoff) {
                Vector rep = new Vector(obstacle, getCopterLocation());
                rep.normalize();
                double dot = rep.dot(headingVec);
                double angle = Math.acos(Math.max(-1.0, Math.min(1.0, dot / (rep.magnitude() * headingVec.magnitude()))));
                double directionFactor = Math.max(Math.cos(mu * angle), dirRatio);
                Vector omni = rep.scaledCopy(1.0);
                applyMagnitudeFunction(omni, dist);
                if (communication.isUAVHovering(obstacle)) omni.scalarProduct(1.2);
                total = Vector.add(total, rep.scaledCopy(directionFactor));
                total = Vector.add(total, omni);
            }
        }
        if (total.magnitude() > maxspeed * 0.8) {
            total.normalize();
            total.scalarProduct(maxspeed * 0.8);
        }
        return total;
    }

    private void sendVelocityCommand(Vector velocity) {
        if (velocity.magnitude() > maxspeed) {
            velocity.normalize();
            velocity.scalarProduct(maxspeed);
        }
        moveUAV(velocity);
    }

    private void moveUAV(Vector v) {
        copter.moveTo(v.y, v.x, 0);
    }

    private Location3DUTM getCopterLocation() {
        return new Location3DUTM(copter.getLocationUTM(), copter.getAltitude());
    }

    private double getHeading() {
        return Math.toRadians(copter.getHeading());
    }

    private Location3DUTM getLastTarget() {
        return waypoints.isEmpty() ? getCopterLocation() : waypoints.peek();
    }

    private void applyMagnitudeFunction(Vector v, double distance) {
        double strength = Math.max(1.0 - (distance / magneticSwarmRecSimProperties.frd), 0.0);
        v.scalarProduct(strength);
    }
}
