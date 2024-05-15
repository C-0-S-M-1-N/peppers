package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.Parts.MotionProfile;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

@Config
public class ElevatorArm implements Part {

    private AutoServo virtual1, turret, rotation;
    private MotionProfile armProfile = new MotionProfile(4000, 2000);
    private double currentArmAngle = 0, defaultTouretDegrees = 175, imuResetedAngle = 0;
    public ElevatorArm(){
        virtual1 = new AutoServo(SERVO_PORTS.S4, 82.f/355.f, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
        rotation = new AutoServo(SERVO_PORTS.S1,  180.f/355.f,false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);

        turret = new AutoServo(SERVO_PORTS.S5, 0, false, Hubs.EXPANSION_HUB, AutoServo.TYPE.AXON);

        turret.setAngle(defaultTouretDegrees);
        virtual1.setAngle(185);
        rotation.setAngle(0);

        virtual1.update();
        rotation.update();
        turret.update();
    }
    public void setArmAngle(double angle){

        armProfile.startMotion(currentArmAngle, angle);
        currentArmAngle = angle;
    }
    private long timePivot = 0, timeElapsed = 0;
    public void setPivotAngle(double angle, long time){
        timePivot = time;
        timeElapsed = System.currentTimeMillis();
    }
    public void setPivotAngle(double angle){
        setPivotAngle(angle, 0);
    }

    public void setOrientation(double angle){
        while(angle > 180) angle -= 360;
        while(angle < -180) angle += 360;

        if(Math.abs(angle) > 30) angle = 30 * Math.signum(angle);

        turret.setAngle(angle + defaultTouretDegrees);
    }
    public void setPixelRotation(double angle){
        rotation.setAngle(angle);
    }
    public double getPixelRotation(){ return rotation.getAngle(); }
    public double getArmAngle(){
        return currentArmAngle;
    }
    public double getLiveArmAngle(){
        return armProfile.getPosition();
    }

    public boolean reachedStationary(){
        return armProfile.motionEnded();
    }
    @Override
    public void update(){
        virtual1.setAngle(armProfile.getPosition());
        rotation.update();

    }
    @Override
    public void update_values(){
        armProfile.update();
        virtual1.update();
        turret.update();
    }
    @Override
    public void runTelemetry(){
        ControlHub.telemetry.addData("arm angle", getArmAngle());
        ControlHub.telemetry.addData("turret angle", turret.getAngle());
    }

}