package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

public class OutTakeExtensionModule {
    private AutoServo servo1, servo2;

    public static double retractS1 = 0, retractS2 = 0,
                         extendS1 = 0, extendS2 = 0;
    public OutTakeExtensionModule(SERVO_PORTS s0, SERVO_PORTS s1){
        servo1 = new AutoServo(s0, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
        servo2 = new AutoServo(s1, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
    }

    public boolean isExtended(){
        return servo1.getAngle() == extendS1;
    }
    public boolean isRetracted(){
        return servo1.getAngle() == retractS1;
    }

    public void update(){
        servo1.update();
        servo2.update();
    }

    public void extend(){
        servo1.setAngle(extendS1);
        servo2.setAngle(extendS2);
        update();
    }
    public void retract(){
        servo1.setAngle(retractS1);
        servo2.setAngle(retractS2);
        update();
    }
}
