package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Exceptions.OverTheLimitException;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.utils.AutoMotor;
import org.firstinspires.ftc.teamcode.utils.AutoServo;
import org.opencv.core.Mat;

@Config
public class ElevatorArm implements Part {

    private Telemetry telemetry;
    private static AutoServo virtual1, virtual2;
    private double angle, position;

    public ElevatorArm(HardwareMap hm, Telemetry tele){
        telemetry = tele;
        virtual1 = new AutoServo(hm.get(Servo.class, "virtual1"), false, 0, AutoServo.type.DS);
        virtual2 = new AutoServo(hm.get(Servo.class, "virtual2"), true, 0, AutoServo.type.DS);

    }

    @Override
    public void update(){
        virtual1.update();
        virtual2.update();
    }
    @Override
    public void update_values(){
        // nothing to do here :)
    }
    public double getPosition(){
        return virtual1.getPosition();
    }
    public void setPosition(double p){
        position = p;
        virtual1.setPosition(p);
        virtual2.setPosition(p);
    }

    public void setAngle(double a){
        angle = a;
    }

    @Override
    public void runTelemetry(){
        telemetry.addData("angle", angle);
    }
}
