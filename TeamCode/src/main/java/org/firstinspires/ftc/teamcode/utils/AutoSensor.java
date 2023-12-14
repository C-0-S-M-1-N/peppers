package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoSensor{
    private RevColorSensorV3 sensor;
    private double trashHold, disRead;
    private boolean Detected;


    public AutoSensor(RevColorSensorV3 s, double distTrashHold){
        sensor = s;
        trashHold = distTrashHold;
    }

    public void update(){
        disRead = sensor.getDistance(DistanceUnit.MM);

        if(disRead < trashHold)
            Detected = true;
    }

    public boolean ObjDetected(){
        return Detected;
    }

}
