package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

@Config
public class OutTakeExtensionModule {
    private AutoServo servo1, servo2;

    public static double retractS1 = 50, retractS2 = 30,
                         extendS1 = 215, extendS2 = 220;
    public OutTakeExtensionModule(){
        servo1 = new AutoServo(SERVO_PORTS.S1, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
        servo2 = new AutoServo(SERVO_PORTS.S4, 0, false, Hubs.EXPANSION_HUB, AutoServo.TYPE.AXON);
        retract();
    }

    public void update(){
        servo1.update(); servo2.update();
    }

    public void extend(){
        servo1.setAngle(extendS1);
        servo2.setAngle(extendS1);
        update();
    }
    public void retract(){
        servo1.setAngle(retractS1);
        servo2.setAngle(retractS1);
        update();
    }
    public double getExtensionAngle(){
        double value = ControlHub.ExtensionEncoder.getVoltage();
        return value / 3.3;
    }
    public boolean reachedStationary(){
        return getExtensionAngle() == extendS1 || getExtensionAngle() == retractS1;
    }
    public boolean isExtended(){
        return getExtensionAngle() <= 0.64;
    }
    public boolean isRetracted(){
        return getExtensionAngle() >= 0.8;
    }
}
