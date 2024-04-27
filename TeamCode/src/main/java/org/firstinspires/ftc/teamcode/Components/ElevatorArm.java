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

    private AutoServo virtual1, virtual2, pivot, turret;
    private MotionProfile armProfile = new MotionProfile(4000, 2000);
    private double currentArmAngle = 0, defaultTouretDegrees = 193, imuResetedAngle = 0;
    public ElevatorArm(){
        virtual1 = new AutoServo(SERVO_PORTS.S0, 10.f/355, false, Hubs.EXPANSION_HUB, AutoServo.TYPE.AXON);
        virtual2 = new AutoServo(SERVO_PORTS.S2, 10.f/355, true, Hubs.EXPANSION_HUB, AutoServo.TYPE.AXON);

        pivot = new AutoServo(SERVO_PORTS.S5, 15.f/355, true, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);

        turret = new AutoServo(SERVO_PORTS.S1, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);

        turret.setAngle(defaultTouretDegrees);
        virtual1.setAngle(0);
        virtual2.setAngle(0);
        pivot.setAngle(0);

        virtual1.update();
        virtual2.update();
        pivot.update();
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
        pivot.setAngle(angle);
    }
    public void setPivotAngle(double angle){
        setPivotAngle(angle, 0);
    }

    public void setOrientation(double angle){
        while(angle > 180) angle -= 360;
        while(angle < -180) angle += 360;

        if(Math.abs(angle) > 90) angle = 90 * Math.signum(angle);

        turret.setAngle(angle + defaultTouretDegrees);
    }
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
        if(System.currentTimeMillis() - timeElapsed >= timePivot)
            pivot.update();
        virtual1.setAngle(armProfile.getPosition());
        virtual2.setAngle(armProfile.getPosition());

    }
    @Override
    public void update_values(){
        armProfile.update();
        virtual1.update();
        virtual2.update();
        pivot.update();
        turret.update();
    }
    @Override
    public void runTelemetry(){

    }

}