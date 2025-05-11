package com.protocols.magneticSwarmRec.pojo;

import com.api.communications.HighlevelCommLink;
import es.upv.grc.mapper.Location3DUTM;
import org.json.JSONObject;

import java.util.HashMap;
import java.util.Map;

public class Message {

    private static final int LOCATION = 1;

    public static JSONObject location(int numUAV, Location3DUTM loc) {
        JSONObject msg = new JSONObject();
        msg.put(HighlevelCommLink.Keywords.SENDERID, numUAV);
        msg.put(HighlevelCommLink.Keywords.MESSAGEID, LOCATION);

        JSONObject location = new JSONObject();
        location.put("x", loc.x);
        location.put("y", loc.y);
        location.put("z", 0); // Z fija o ignorada (2D)
        msg.put("location", location);

        return msg;
    }

    public static Map<String, Object> location(int numUAV) {
        Map<String, Object> mandatoryFields = new HashMap<>();
        mandatoryFields.put(HighlevelCommLink.Keywords.MESSAGEID, LOCATION);
        return mandatoryFields;
    }

    public static Location3DUTM processLocation(JSONObject msg) {
        if (msg != null) {
            JSONObject locationJSON = msg.getJSONObject("location");
            double x = locationJSON.getDouble("x");
            double y = locationJSON.getDouble("y");
            double z = 0; // Ignorar valor original si lo hay
            return new Location3DUTM(x, y, z);
        }
        return null;
    }
}
