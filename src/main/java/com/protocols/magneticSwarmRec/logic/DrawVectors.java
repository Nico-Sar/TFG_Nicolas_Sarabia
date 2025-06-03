package com.protocols.magneticSwarmRec.logic;

import com.api.ArduSim;
import com.api.copter.Copter;
import com.protocols.magneticSwarmRec.pojo.Vector;
import com.setup.Param;
import es.upv.grc.mapper.*;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class DrawVectors {

    private final List<AtomicReference<DrawableLinesGeo>> vectorLines;
    private final Copter copter;

    public DrawVectors(Copter copter) {
        this.copter = copter;
        this.vectorLines = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            vectorLines.add(new AtomicReference<>());
        }
    }

    public void update(Vector attraction, Vector repulsion, Vector resulting) {
        if (Param.role == ArduSim.SIMULATOR_GUI) {
            try {
                removeOldDrawings();
                Location2DGeo start = copter.getLocationUTM().getGeo();

                drawVector(start, attraction, Color.BLUE, 0);   // Atracción
                drawVector(start, repulsion, Color.RED, 1);     // Repulsión
                drawVector(start, resulting, Color.GREEN, 2);   // Resultante

            } catch (LocationNotReadyException | GUIMapPanelNotReadyException e) {
                System.err.println("❌ Error al dibujar vectores: " + e.getMessage());
            }
        }
    }

    /**
     * Dibuja un punto visual en el mapa como una línea de longitud 0.
     */
    public void drawPoint(Location3DUTM point) {
        try {
            List<Location2DGeo> list = new ArrayList<>();
            Location2DGeo geo = point.getGeo();
            list.add(geo);
            list.add(geo); // misma coordenada = línea invisible = punto visual
            Mapper.Drawables.addLinesGeo(3, list, Color.MAGENTA, new BasicStroke(3f));
        } catch (Exception e) {
            System.err.println("❌ Error al dibujar punto: " + e.getMessage());
        }
    }

    /**
     * Alias de drawPoint: para marcar puntos de formación visualmente.
     */
    public void drawCircle(Location3DUTM center) {
        drawPoint(center);
    }

    private void removeOldDrawings() {
        for (AtomicReference<DrawableLinesGeo> lineRef : vectorLines) {
            DrawableLinesGeo current = lineRef.getAndSet(null);
            if (current != null) {
                try {
                    Mapper.Drawables.removeDrawable(current);
                } catch (GUIMapPanelNotReadyException e) {
                    System.err.println("❌ Error al eliminar dibujo anterior: " + e.getMessage());
                }
            }
        }
    }

    private void drawVector(Location2DGeo start, Vector vector, Color color, int index)
            throws LocationNotReadyException, GUIMapPanelNotReadyException {

        Location2DUTM stop = copter.getLocationUTM();
        stop.x += vector.x;
        stop.y += vector.y;

        List<Location2DGeo> vectorGeo = new ArrayList<>();
        vectorGeo.add(start);
        vectorGeo.add(stop.getGeo());

        DrawableLinesGeo line = Mapper.Drawables.addLinesGeo(3, vectorGeo, color, new BasicStroke(2f));
        vectorLines.get(index).set(line);
    }
    public void drawPolygon(List<Location3DUTM> points) {
        try {
            List<Location2DGeo> geoPoints = new ArrayList<>();
            for (Location3DUTM utm : points) {
                geoPoints.add(utm.getGeo());
            }
            // Cierra el contorno uniendo último con primero
            geoPoints.add(geoPoints.get(0));

            Mapper.Drawables.addLinesGeo(3, geoPoints, Color.ORANGE, new BasicStroke(1.5f));
        } catch (Exception e) {
            System.err.println("❌ Error al dibujar polígono: " + e.getMessage());
        }
    }

}
