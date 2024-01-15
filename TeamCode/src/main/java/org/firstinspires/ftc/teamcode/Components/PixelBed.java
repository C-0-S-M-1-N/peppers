package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Exceptions.OverTheLimitException;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

@Config
public class PixelBed implements Part {

    public static boolean Disable = false;
    private static AutoServo pivot;
    private Telemetry telemetry;

    public PixelBed(Telemetry tele){
        pivot = new AutoServo(SERVO_PORTS.S5,false, true, 0, AutoServo.type.AXON);
        telemetry = tele;
    }

    @Override
    public void update(){
        if(Disable) return;
        pivot.update();
    }
    public void setBedAngle(double angle){
        pivot.setAngle(angle);
    }
    public void reset(){
        pivot.setAngle(0);
    }
    public double getPivotAngle(){
        return pivot.getAngle();
    }
    @Override
    public void update_values(){

    }
    @Override
    public void runTelemetry(){
        telemetry.addData("pivot angle", pivot.getAngle());
    }
}
