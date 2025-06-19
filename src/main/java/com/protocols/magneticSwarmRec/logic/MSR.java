package com.protocols.magneticSwarmRec.logic;

import com.api.API;
import com.api.copter.Copter;
import com.api.copter.TakeOff;
import com.api.copter.TakeOffListener;
import com.api.swarm.formations.Formation;
import com.api.swarm.formations.FormationFactory;
import com.protocols.magneticSwarmRec.gui.magneticSwarmRecSimProperties;
import com.protocols.magneticSwarmRec.pojo.HungarianAlgorithm;
import com.protocols.magneticSwarmRec.pojo.UAVLogger;
import com.protocols.magneticSwarmRec.pojo.Vector;
import es.upv.grc.mapper.Location3DGeo;
import es.upv.grc.mapper.Location3DUTM;
import org.javatuples.Pair;

import java.util.*;
import java.util.concurrent.CyclicBarrier;

public class MSR extends Thread {
    // Variables
    private Queue<Location3DUTM> waypoints;
    private final Copter copter;
    private final int numUAV, numUAVs;
    private final Communication communication;
    private final DrawVectors drawVectors;
    private static CyclicBarrier barrier, landingBarrier;
    private int attempts = 0;

    // Parámetros configurables
    private final double maxspeed;
    private final double threshold;
    private final double minFlightDistance;
    private final double frdOmni;
    private final double omniRepulsionStrength;
    private final double frdDir;
    private final double dirRepulsionStrength;
    private final double dirFactor;
    private final double maxRepulsionFactor;
    private final double weightAttraction;
    private final double weightRepulsion;
    private final double angleWeight;
    private final double neighborDetectionRadius;

    private UAVState state = UAVState.MOVING;

    private enum UAVState {MOVING, HOVERING, LANDING}

    private Location3DUTM centerUTM, finalTarget;
    private final UAVLogger logger;
    private long formationStartTime = -1;

    private static final boolean debugFormacion = true;

    // Constructor
    public MSR(int numUAV) {
        this.numUAV = numUAV;
        this.numUAVs = API.getArduSim().getNumUAVs();
        this.copter = API.getCopter(numUAV);
        this.drawVectors = new DrawVectors(copter);
        this.communication = new Communication(numUAV);
        this.waypoints = new LinkedList<>();
        this.logger = new UAVLogger(numUAV);

        // Parámetros desde SimProperties
        this.maxspeed = magneticSwarmRecSimProperties.speed;
        this.threshold = magneticSwarmRecSimProperties.threshold;
        this.minFlightDistance = magneticSwarmRecSimProperties.minFlightDistance;
        this.frdOmni = magneticSwarmRecSimProperties.frdOmni;
        this.omniRepulsionStrength = magneticSwarmRecSimProperties.omniRepulsionStrength;
        this.frdDir = magneticSwarmRecSimProperties.frdDir;
        this.dirRepulsionStrength = magneticSwarmRecSimProperties.dirRepulsionStrength;
        this.dirFactor = magneticSwarmRecSimProperties.dirFactor;
        this.weightAttraction = magneticSwarmRecSimProperties.weightAttraction;
        this.weightRepulsion = magneticSwarmRecSimProperties.weightRepulsion;
        this.maxRepulsionFactor = magneticSwarmRecSimProperties.maxRepulsionFactor;
        this.angleWeight = magneticSwarmRecSimProperties.angleWeight;
        this.neighborDetectionRadius = magneticSwarmRecSimProperties.neighborDetectionRadius;

        synchronized (MSR.class) {
            if (barrier == null) {
                barrier = new CyclicBarrier(numUAVs);
                landingBarrier = new CyclicBarrier(numUAVs);
            }
        }
    }

    private static class RepulsionResult {
        public final Vector repulsion;
        public final double slowdown;

        public RepulsionResult(Vector repulsion, double slowdown) {
            this.repulsion = repulsion;
            this.slowdown = slowdown;
        }
    }

    @Override
    public void run() {
        takeoff();
        communication.start();
        switchToFlyingFormation();
        formationStartTime = System.currentTimeMillis();

        while (true) {
            switch (state) {
                case MOVING:
                    if (waypoints.isEmpty()) {
                        state = UAVState.HOVERING;
                        break;
                    }

                    Location3DUTM destino = waypoints.peek();
                    Location3DUTM current = getCopterLocation();
                    double dist = current.distance3D(destino);

                    RepulsionResult result = calculateRepulsionAndSlowdown();
                    Vector attraction = getAttractionVector(result.slowdown);
                    Vector repulsion = result.repulsion;
                    penalizeAttractionIfNeeded(attraction, repulsion, dist);

                    Vector total = Vector.add(attraction, repulsion);
                    checkPotentialCollision(total);
                    sendVelocityCommand(total);

                    // Evaluación de llegada al waypoint
                    if (dist < threshold && isUAVStable()) {
                        waypoints.poll();
                        attempts = 0;
                    } else if (dist <= minFlightDistance / 2 && attempts > 60) {
                        System.out.printf("⚠️ UAV %d está cerca (%.2f m) pero aún se mueve\n", numUAV, dist);
                        waypoints.poll();
                        attempts = 0;
                    } else {
                        attempts++;
                    }
                    break;

                case HOVERING:
                    moveUAV(new Vector(0, 0));
                    communication.setHovering(true);
                    if (communication.allUAVsHovering()) {
                        try {
                            System.out.printf("🛑 UAV %d en posición final. Esperando para aterrizar...\n", numUAV);
                            API.getArduSim().sleep(3000);  // Esto puede considerarse parametizable en el futuro
                            landingBarrier.await();
                            logger.setFlightEndTime(System.currentTimeMillis());
                            logger.setFormationDuration(System.currentTimeMillis() - formationStartTime);
                            state = UAVState.LANDING;
                        } catch (Exception ignored) {
                        }
                    }
                    break;

                case LANDING:
                    communication.stopCommunication();
                    Location3DUTM finalPos = getCopterLocation();
                    System.out.printf("✅ UAV %d aterriza en (%.2f, %.2f) | Distancia al objetivo = %.2f\n",
                            numUAV, finalPos.x, finalPos.y, finalPos.distance3D(finalTarget));
                    copter.land();
                    logger.saveToFile();
                    return;
            }

            // Tiempo de ciclo del bucle de control
            API.getArduSim().sleep(100);  // Este valor puede quedar fijo salvo que se requiera testing fino
        }
    }


    private RepulsionResult calculateRepulsionAndSlowdown() {
        Vector total = new Vector();
        double slowdown = 1.0;

        Location3DUTM currentPos = getCopterLocation();
        Vector headingVec = headingVec();

        for (Pair<Location3DUTM, Double> obs : communication.getObstaclesWithHeading()) {
            Location3DUTM obstacle = obs.getValue0();
            double dist = currentPos.distance3D(obstacle);

            if (dist > Math.max(frdOmni, frdDir)) continue;

            Vector rep = new Vector(obstacle, currentPos);
            rep.normalize();

            // Omnidireccional
            if (dist <= frdOmni) {
                double omniMag = (dist <= minFlightDistance)
                        ? omniRepulsionStrength
                        : omniRepulsionStrength * (1 - Math.pow((dist - minFlightDistance) / (frdOmni - minFlightDistance), 2));

                total = Vector.add(total, rep.scaledCopy(omniMag));
            }

            // Direccional
            if (dist <= frdDir) {
                double dot = rep.dot(headingVec);
                double angle = Math.acos(Math.max(-1.0, Math.min(1.0,
                        dot / (rep.magnitude() * headingVec.magnitude()))));

                // Aplica repulsión solo si el obstáculo está en la parte delantera (cos(angle) > 0)
                double directionFactor = Math.cos(dirFactor * angle);
                if (directionFactor <= 0) directionFactor = 0;

                // Cálculo de magnitud de la repulsión direccional
                double dirMag = (dist > minFlightDistance)
                        ? dirRepulsionStrength * (1 - Math.pow((dist - minFlightDistance) / (frdDir - minFlightDistance), 2))
                        : 0.0;

                Vector directional = rep.scaledCopy(dirMag * directionFactor);
                total = Vector.add(total, directional);

                // Ralentización si el obstáculo está al frente
                if (dot > 0 && dist > 0) {
                    double proximityFactor = 1.0 - (dist / frdDir);
                    double penalty = proximityFactor * dot;
                    slowdown *= (1.0 - penalty * 0.5);
                }
            }

        }

        total.scalarProduct(weightRepulsion);

        double maxRepulsion = maxspeed * maxRepulsionFactor;
        if (total.magnitude() > maxRepulsion) {
            total.normalize();
            total.scalarProduct(maxRepulsion);
        }

        return new RepulsionResult(total, Math.max(0.3, slowdown));
    }

    private Vector getAttractionVector(double slowdownFactor) {
        Location3DUTM current = getCopterLocation();
        Location3DUTM target = waypoints.peek();
        double dist = current.distance3D(target);

        // Dirección normalizada hacia el objetivo
        Vector attraction = new Vector(current, target);
        attraction.normalize();

        // Cálculo de velocidad adaptativa por vecinos y distancia
        int neighbors = countNeighborsWithinRadius();
        double adaptiveMaxSpeed = calculateAdaptiveMaxSpeed(neighbors, dist);
        double speed = calculateSpeedBasedOnNeighbors(adaptiveMaxSpeed, neighbors);
        speed = applyDistanceSpeedLimit(speed, adaptiveMaxSpeed, dist);

        // Desaceleración por repulsión direccional
        speed *= slowdownFactor;

        // Suavizado final por cercanía al objetivo
        if (dist < threshold) {
            double smoothingRadius = 1.5 * threshold;  // puede ajustarse si lo prefieres más o menos brusco
            speed *= (dist / smoothingRadius);
            if (speed < 0.05) speed = 0;  // umbral mínimo para evitar vibraciones/residuos
        }

        if (speed == 0) return new Vector(0, 0);

        attraction.scalarProduct(speed);
        return attraction;
    }


    private void penalizeAttractionIfNeeded(Vector attraction, Vector repulsion, double dist) {
        double repulsionMag = repulsion.magnitude();

        // Si la repulsión es significativa y estamos aún a cierta distancia del objetivo
        if (repulsionMag > 0.2 * maxspeed && dist > minFlightDistance) {
            double penaltyFactor = Math.max(0.5, 1.0 - repulsionMag / maxspeed);

            attraction.scalarProduct(penaltyFactor);

            System.out.printf(
                    "🐢 UAV %d reduce velocidad por repulsión: factor=%.2f | Repulsión=%.2f m/s\n",
                    numUAV, penaltyFactor, repulsionMag
            );
        }
    }


    private void sendVelocityCommand(Vector velocity) {
        // Limitar magnitud máxima según maxspeed
        if (velocity.magnitude() > maxspeed) {
            velocity.normalize();
            velocity.scalarProduct(maxspeed);
        }

        moveUAV(velocity);
    }

    private void switchToFlyingFormation() {
        Formation.Layout layout = Formation.Layout.valueOf(magneticSwarmRecSimProperties.flyingFormation.toUpperCase());
        Formation target = FormationFactory.newFormation(layout);

        double alt = magneticSwarmRecSimProperties.altitude;
        target.init(numUAVs, magneticSwarmRecSimProperties.flyingDistance);

        // Calcular centro de la formación
        double latSum = 0.0, lonSum = 0.0;
        for (int i = 0; i < numUAVs; i++) {
            latSum += API.getCopter(i).getLocation().getLatitude();
            lonSum += API.getCopter(i).getLocation().getLongitude();
        }
        double centerLat = latSum / numUAVs;
        double centerLon = lonSum / numUAVs;
        centerUTM = Location3DGeo.getUTM(centerLat, centerLon, alt);

        // Visualización de la formación generada
        if (debugFormacion && numUAV == 0) {
            System.out.println("📌 Verificación visual de formación generada:");
            List<Location3DUTM> polygonPoints = new ArrayList<>();
            for (int i = 0; i < numUAVs; i++) {
                Location3DUTM pos = target.get3DUTMLocation(centerUTM, i);
                System.out.printf("🔘 Pos %d → (%.2f, %.2f)\n", i, pos.x, pos.y);
                drawVectors.drawCircle(pos);
                polygonPoints.add(pos);
            }
            drawVectors.drawPolygon(polygonPoints);
        }

        // Asignación de UAVs a posiciones objetivo
        List<Location3DUTM> currentPositions = new ArrayList<>();
        List<Location3DUTM> targetPositions = new ArrayList<>();
        for (int i = 0; i < numUAVs; i++) {
            currentPositions.add(new Location3DUTM(API.getCopter(i).getLocationUTM(), alt));
            targetPositions.add(target.get3DUTMLocation(centerUTM, i));
        }

        int[] assignment = assignWithMinimalInterference(currentPositions, targetPositions);

        if (layout == Formation.Layout.LINEAR) {
            Location3DUTM finalWaypoint = targetPositions.get(assignment[numUAV]);
            Location3DUTM currentPos = currentPositions.get(numUAV);
            Location3DUTM intermediateWaypoint = new Location3DUTM(finalWaypoint.x, currentPos.y, alt);

            waypoints.clear();
            waypoints.add(intermediateWaypoint);
            waypoints.add(finalWaypoint);
            finalTarget = finalWaypoint;

            System.out.printf("📦 UAV %d - Desde (%.2f, %.2f)\n", numUAV, currentPos.x, currentPos.y);
            System.out.printf("➡ UAV %d - Waypoint intermedio: (%.2f, %.2f)\n", numUAV, intermediateWaypoint.x, intermediateWaypoint.y);
            System.out.printf("🎯 UAV %d - Waypoint FINAL: (%.2f, %.2f)\n", numUAV, finalWaypoint.x, finalWaypoint.y);
        } else {
            for (int i = 0; i < numUAVs; i++) {
                double dist = currentPositions.get(i).distance3D(targetPositions.get(assignment[i]));
                System.out.printf("📍 Asignación: UAV %d -> Posición %d | Distancia = %.2f\n", i, assignment[i], dist);
            }

            Location3DUTM targetPos = targetPositions.get(assignment[numUAV]);
            Location3DUTM current = getCopterLocation();
            finalTarget = targetPos;

            System.out.printf("📦 UAV %d - Desde (%.2f, %.2f)\n", numUAV, current.x, current.y);
            System.out.printf("🎯 UAV %d - Waypoint FINAL: (%.2f, %.2f)\n", numUAV, targetPos.x, targetPos.y);

            waypoints.clear();
            waypoints.add(targetPos);
        }
    }


    private void moveUAV(Vector v) {
        copter.moveTo(v.y, v.x, 0);  // y = norte-sur, x = este-oeste
    }

    private Location3DUTM getCopterLocation() {
        return new Location3DUTM(copter.getLocationUTM(), copter.getAltitude());
    }

    private boolean isUAVStable() {
        double[] speed = copter.getSpeedComponents();
        double vx = speed[0];
        double vy = speed[1];
        double horizontalSpeed = Math.sqrt(vx * vx + vy * vy);
        return horizontalSpeed < 0.5;
    }

    private Vector headingVec() {
        return new Vector(Math.cos(getHeading()), Math.sin(getHeading()));
    }

    private double getHeading() {
        return Math.toRadians(copter.getHeading());
    }

    private int countNeighborsWithinRadius() {
        int count = 0;
        Location3DUTM current = getCopterLocation();
        for (Pair<Location3DUTM, Double> obs : communication.getObstaclesWithHeading()) {
            if (current.distance3D(obs.getValue0()) <= neighborDetectionRadius) count++;
        }
        return count;
    }


    private double calculateAdaptiveMaxSpeed(int neighbors, double dist) {
        // Posible parámetro extra si se quiere ajustar "dist > 10"
        if (neighbors == 0 && dist > 10) return maxspeed * 1.2;
        return maxspeed;
    }

    private double calculateSpeedBasedOnNeighbors(double adaptiveMaxSpeed, int neighbors) {
        if (neighbors == 0) return adaptiveMaxSpeed;
        if (neighbors <= 4) return adaptiveMaxSpeed * 0.75;
        if (neighbors <= 8) return adaptiveMaxSpeed * 0.5;
        return adaptiveMaxSpeed * 0.3;
    }

    private double applyDistanceSpeedLimit(double speed, double adaptiveMaxSpeed, double dist) {
        if (dist > 50) return Math.min(speed, adaptiveMaxSpeed);
        if (dist > 30) return Math.min(speed, adaptiveMaxSpeed * 0.3);
        if (dist > 15) return Math.min(speed, adaptiveMaxSpeed * 0.15);
        if (dist > 7) return Math.min(speed, adaptiveMaxSpeed * 0.10);
        return Math.min(speed, adaptiveMaxSpeed * 0.05);
    }

    private int[] assignWithMinimalInterference(List<Location3DUTM> current, List<Location3DUTM> target) {
        int n = current.size();
        double[][] costMatrix = new double[n][n];

        for (int i = 0; i < n; i++) {
            Location3DUTM from = current.get(i);
            for (int j = 0; j < n; j++) {
                Location3DUTM to = target.get(j);
                double dist = from.distance3D(to);

                double dx = to.x - from.x;
                double dy = to.y - from.y;

                double headingRad = Math.toRadians(API.getCopter(i).getHeading());
                double dirX = Math.cos(headingRad);
                double dirY = Math.sin(headingRad);

                double dot = dx * dirX + dy * dirY;
                double magA = Math.hypot(dx, dy);
                double magB = Math.hypot(dirX, dirY);
                double angle = Math.acos(Math.max(-1.0, Math.min(1.0, dot / (magA * magB))));

                costMatrix[i][j] = dist + angleWeight * angle;  // angleWeight ya parametrizado
            }
        }

        HungarianAlgorithm hungarian = new HungarianAlgorithm(costMatrix);
        return hungarian.execute();
    }

    private void takeoff() {
        TakeOff takeOff = copter.takeOff(magneticSwarmRecSimProperties.altitude, new TakeOffListener() {
            public void onCompleteActionPerformed() {
            }

            public void onFailure() {
            }
        });
        takeOff.start();
        try {
            takeOff.join();
            barrier.await();
            logger.setFlightStartTime(System.currentTimeMillis());
        } catch (Exception ignored) {
        }
    }

    private void checkPotentialCollision(Vector velocity) {
        for (Pair<Location3DUTM, Double> obs : communication.getObstaclesWithHeading()) {
            Location3DUTM other = obs.getValue0();
            double dist = getCopterLocation().distance3D(other);
            if (dist < minFlightDistance) {
                System.out.printf("❌ Posible colisión: UAV %d muy cerca de otro UAV (%.2f m)\n", numUAV, dist);
            }
        }
    }
}