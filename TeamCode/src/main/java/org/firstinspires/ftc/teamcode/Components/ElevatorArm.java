package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

@Config
public class ElevatorArm implements Part {

    private AutoServo virtual1, virtual2, pivot, turret;
    private ElapsedTime TMP_arm = new ElapsedTime(),
                        TMP_pivot = new ElapsedTime(),
                        TMP_turret = new ElapsedTime();

    public ElevatorArm(){
        virtual1 = new AutoServo(SERVO_PORTS.S0, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
        virtual2 = new AutoServo(SERVO_PORTS.S1, 0, true, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);

        pivot = new AutoServo(SERVO_PORTS.S2, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);

        turret = new AutoServo(SERVO_PORTS.S3, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);



        turret.setAngle(90);
        virtual1.setAngle(0);
        virtual2.setAngle(0);
        pivot.setAngle(0);

        virtual1.update();
        virtual2.update();
        pivot.update();
        turret.update();
    }

    public void setArmAngle(double angle){
        virtual1.setAngle(angle);
        virtual2.setAngle(angle);
        TMP_arm.reset();
    }
    public void setPivotAngle(double angle){
        pivot.setAngle(angle);
        TMP_pivot.reset();
    }

    public void setOrientation(double angle){
        turret.setAngle(angle);
        TMP_turret.reset();
    }

    public boolean reachedTargetArmPosition(){
        return TMP_arm.seconds() >= 1;
    }
    public boolean reachedTargetPivotPosition(){
        return TMP_pivot.seconds() >= 0.5;
    }
    public boolean reachedTargetTourretPosition(){
        return TMP_turret.seconds() >= 1.5;
    }

    public boolean reachedStationary(){
        return reachedTargetTourretPosition() && reachedTargetArmPosition() && reachedTargetPivotPosition();
    }

    @Override
    public void update(){

    }
    @Override
    public void update_values(){

    }
    @Override
    public void runTelemetry(){

    }

}