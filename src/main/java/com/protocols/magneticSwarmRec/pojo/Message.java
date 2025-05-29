package com.protocols.magneticSwarmRec.pojo;

import com.api.communications.HighlevelCommLink;
import es.upv.grc.mapper.Location3DUTM;
import org.javatuples.Pair;
import org.json.JSONObject;

import java.util.HashMap;
import java.util.Map;

public class Message {

    private static final int LOCATION = 1;

    // Envío clásico (sin estado)
    public static JSONObject location(int numUAV, Location3DUTM loc, double heading) {
        JSONObject msg = new JSONObject();
        msg.put(HighlevelCommLink.Keywords.SENDERID, numUAV);
        msg.put(HighlevelCommLink.Keywords.MESSAGEID, LOCATION);

        JSONObject location = new JSONObject();
        location.put("x", loc.x);
        location.put("y", loc.y);
        location.put("z", 0); // 2D
        msg.put("location", location);
        msg.put("heading", heading);

        return msg;
    }

    // Nueva versión: incluye estado (MOVING, HOVERING, etc.)
    public static JSONObject location(int numUAV, Location3DUTM loc, double heading, String state) {
        JSONObject msg = location(numUAV, loc, heading); // llama al método anterior
        msg.put("state", state); // estado adicional
        return msg;
    }

    public static Map<String, Object> location(int numUAV) {
        Map<String, Object> mandatoryFields = new HashMap<>();
        mandatoryFields.put(HighlevelCommLink.Keywords.MESSAGEID, LOCATION);
        return mandatoryFields;
    }

    public static Pair<Location3DUTM, Double> processLocationWithHeading(JSONObject msg) {
        if (msg != null) {
            JSONObject locationJSON = msg.getJSONObject("location");
            double x = locationJSON.getDouble("x");
            double y = locationJSON.getDouble("y");
            double z = 0; // Ignorar Z

            double heading = msg.has("heading") ? msg.getDouble("heading") : 0.0;
            return Pair.with(new Location3DUTM(x, y, z), heading);
        }
        return null;
    }


    public static String getState(JSONObject msg) {
        return msg.optString("state", "MOVING");
    }
}
