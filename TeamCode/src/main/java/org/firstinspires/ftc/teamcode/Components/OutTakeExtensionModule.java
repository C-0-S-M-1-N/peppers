package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AsymetricMotionProfile;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

@Config
public class OutTakeExtensionModule {
    private AutoServo servo1, servo2;

    public static double retractS1 = 50, retractS2 = 30,
                         extendS1 = 205, extendS2 = 220, A = 400, D = 400, MV = 600;
    public static boolean TUNE = false;
    private AsymetricMotionProfile AMP;
    public OutTakeExtensionModule(){
        servo1 = new AutoServo(SERVO_PORTS.S1, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
        servo2 = new AutoServo(SERVO_PORTS.S4, 0, false, Hubs.EXPANSION_HUB, AutoServo.TYPE.AXON);
        AMP = new AsymetricMotionProfile(A, D, MV);
        retract();
    }

    public void update(){
        if(TUNE && AMP.motionEnded()){
            AMP.Acceleration = A;
            AMP.Decceleration = D;
            AMP.MaxVelocity = MV;
        }
        double a = AMP.getPosition();
        servo1.setAngle(a);
        servo2.setAngle(a);
        servo1.update(); servo2.update();
    }

    public void extend(){
        AMP.startMotion(retractS1, extendS1);
    }
    public void retract(){
        AMP.startMotion(extendS1, retractS1);
    }
    public double getExtensionAngle(){
        double value = ControlHub.ExtensionEncoder.getVoltage();
        return value * 360 / 3.3;
    }
    public boolean isExtended(){
        return AMP.targetPosition == extendS1 && AMP.motionEnded();
    }
    public boolean isRetracted(){
        return (AMP.targetPosition == retractS1 || AMP.targetPosition == 0) && AMP.motionEnded();
    }

}
