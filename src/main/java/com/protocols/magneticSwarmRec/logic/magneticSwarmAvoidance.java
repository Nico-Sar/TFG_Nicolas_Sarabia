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
        Formation target = FormationFactory.newFormation(
                Formation.Layout.valueOf(magneticSwarmRecSimProperties.flyingFormation.toUpperCase())
        );
        target.init(numUAVs, magneticSwarmRecSimProperties.flyingDistance);
        double alt = magneticSwarmRecSimProperties.altitude;

        // Calcular centro de la formación
        double latSum = 0.0, lonSum = 0.0;
        for (int i = 0; i < numUAVs; i++) {
            latSum += API.getCopter(i).getLocation().getLatitude();
            lonSum += API.getCopter(i).getLocation().getLongitude();
        }
        double centerLat = latSum / numUAVs;
        double centerLon = lonSum / numUAVs;
        Location3DUTM centerUTM = Location3DGeo.getUTM(centerLat, centerLon, alt);

        // Obtener posiciones actuales y destino
        List<Location3DUTM> currentPositions = new ArrayList<>();
        List<Location3DUTM> targetPositions = new ArrayList<>();
        for (int i = 0; i < numUAVs; i++) {
            currentPositions.add(new Location3DUTM(API.getCopter(i).getLocationUTM(), alt));
            targetPositions.add(target.get3DUTMLocation(centerUTM, i));
        }

        // Asignación según tipo de formación
        int[] assignment;
        if (target.getLayout() == Formation.Layout.CIRCLE) {
            assignment = assignByAngularSort(currentPositions, targetPositions, centerUTM);
        } else {
            assignment = assignAvoidingCrossesHungarian(currentPositions, targetPositions);
        }

        // Mostrar asignación
        for (int i = 0; i < numUAVs; i++) {
            double dist = currentPositions.get(i).distance3D(targetPositions.get(assignment[i]));
            System.out.printf("Asignación: UAV %d -> Posición %d | Distancia = %.2f\n", i, assignment[i], dist);
        }

        // Planificar movimiento del UAV actual
        int assignedIndex = assignment[numUAV];
        Location3DUTM targetPos = targetPositions.get(assignedIndex);
        Location3DUTM current = getCopterLocation();

        double midX = current.x + 0.7 * (targetPos.x - current.x);
        double midY = current.y + 0.7 * (targetPos.y - current.y);
        Location3DUTM intermediate = new Location3DUTM(midX, midY, alt);

        waypoints.clear();
        waypoints.add(intermediate);
        waypoints.add(targetPos);

        System.out.printf("UAV %d - Waypoint INTERMEDIO: (%.2f, %.2f)\n", numUAV, intermediate.x, intermediate.y);
        System.out.printf("UAV %d - Waypoint FINAL: (%.2f, %.2f)\n", numUAV, targetPos.x, targetPos.y);
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
    private int[] assignAvoidingCrossesHungarian(List<Location3DUTM> current, List<Location3DUTM> target) {
        int n = current.size();
        double[][] costMatrix = new double[n][n];

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                costMatrix[i][j] = current.get(i).distance3D(target.get(j));
            }
        }

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                Location3DUTM a1 = current.get(i);
                Location3DUTM a2 = target.get(j);
                for (int k = i + 1; k < n; k++) {
                    for (int l = 0; l < n; l++) {
                        if (j == l) continue;
                        Location3DUTM b1 = current.get(k);
                        Location3DUTM b2 = target.get(l);
                        if (segmentsCross(a1, a2, b1, b2)) {
                            costMatrix[i][j] += 10000;
                            costMatrix[k][l] += 10000;
                            System.out.printf("⚠️ Cruce detectado entre trayectorias: UAV%d → Pos%d y UAV%d → Pos%d\n", i, j, k, l);
                        }
                    }
                }
            }
        }

        return new HungarianAlgorithm(costMatrix).execute();
    }

    private boolean segmentsCross(Location3DUTM a1, Location3DUTM a2, Location3DUTM b1, Location3DUTM b2) {
        return linesIntersect(a1.x, a1.y, a2.x, a2.y, b1.x, b1.y, b2.x, b2.y);
    }

    private boolean linesIntersect(double x1, double y1, double x2, double y2,
                                   double x3, double y3, double x4, double y4) {
        double d1 = direction(x3, y3, x4, y4, x1, y1);
        double d2 = direction(x3, y3, x4, y4, x2, y2);
        double d3 = direction(x1, y1, x2, y2, x3, y3);
        double d4 = direction(x1, y1, x2, y2, x4, y4);
        return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
                ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)));
    }

    private double direction(double xi, double yi, double xj, double yj, double xk, double yk) {
        return (xk - xi) * (yj - yi) - (xj - xi) * (yk - yi);
    }
    private int[] assignByAngularSort(List<Location3DUTM> currentPositions, List<Location3DUTM> targetPositions, Location3DUTM center) {
        int n = currentPositions.size();
        List<Pair<Double, Integer>> anglesCurrent = new ArrayList<>();
        List<Pair<Double, Integer>> anglesTarget = new ArrayList<>();

        for (int i = 0; i < n; i++) {
            double aC = Math.atan2(currentPositions.get(i).y - center.y, currentPositions.get(i).x - center.x);
            double aT = Math.atan2(targetPositions.get(i).y - center.y, targetPositions.get(i).x - center.x);
            anglesCurrent.add(Pair.with(aC, i));
            anglesTarget.add(Pair.with(aT, i));
        }

        anglesCurrent.sort(Comparator.comparingDouble(Pair::getValue0));
        anglesTarget.sort(Comparator.comparingDouble(Pair::getValue0));

        int[] assignment = new int[n];
        for (int i = 0; i < n; i++) {
            int uavId = anglesCurrent.get(i).getValue1();
            int posId = anglesTarget.get(i).getValue1();
            assignment[uavId] = posId;
        }

        return assignment;
    }

}
