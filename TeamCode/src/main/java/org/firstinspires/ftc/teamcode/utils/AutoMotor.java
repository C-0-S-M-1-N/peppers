package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class AutoMotor {
    public enum STATES{
        RESET_0,
        TRIGGER_RESET,
        NORMAL,
        TRIGGER_NORMAL;
    }
    public STATES STATE;
    public boolean reverse = false;
    public DcMotorEx motor;
    private int position, targetPosition, velocity;
    private double power;
    private double currentUsed;

    public AutoMotor(DcMotorEx m, boolean r){
        motor = m;
        reverse = r;

        if(r) motor.setDirection(DcMotorSimple.Direction.REVERSE);
        else motor.setDirection(DcMotorSimple.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorConfigurationType mct = motor.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(mct);


        motor.setTargetPosition(0);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(1);


        STATE = STATES.RESET_0;

    }

    public void update(){
        int prevPos = position;
        position = motor.getCurrentPosition();

        velocity = Math.abs(position - prevPos);
        currentUsed = motor.getCurrent(CurrentUnit.AMPS);

        switch (STATE) {
            case NORMAL:
                motor.setTargetPosition(targetPosition);
                motor.setPower(power);
                break;
            case TRIGGER_RESET:
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                STATE = STATES.RESET_0;
                break;
            case RESET_0:
                motor.setPower(-1);
                if(velocity <= 0.01){
                    STATE = STATES.TRIGGER_NORMAL;
                }
                break;
            case TRIGGER_NORMAL:
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                STATE = STATES.NORMAL;
                targetPosition = 0;

                motor.setTargetPosition(0);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
                break;
        }
    }

    public void setTargetPosition(int pos){ targetPosition = pos; }
    public void setPower(double p){ power = p; }
    public void resetZeroPosition(){
        STATE = STATES.TRIGGER_RESET;
    }

    public int getPosition(){ return position; }
    public int getVelocity(){ return velocity; }
    public double getPower(){ return power; }
    public double getCurrentUsed(){ return currentUsed; }

}
