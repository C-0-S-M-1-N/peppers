package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class AutoMotor {
    public enum STATES{
        RESET_0,
        TRIGGER_RESET,
        NORMAL,
        TRIGGER_NORMAL;
    }
    private STATES STATE;
    public DcMotorEx motor;
    private int position, targetPosition, velocity;
    private double power;

    public AutoMotor(DcMotorEx m){
        motor = m;

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorConfigurationType mct = motor.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(mct);


        motor.setTargetPosition(0);
        motor.setPower(1);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        STATE = STATES.RESET_0;

    }

    public void update(){
        int prevPos = position;
        position = motor.getCurrentPosition();

        velocity = Math.abs(position - prevPos);

        switch (STATE) {
            case NORMAL:
                motor.setTargetPosition(targetPosition);
                motor.setPower(power);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case TRIGGER_RESET:
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                STATE = STATES.RESET_0;
                break;
            case RESET_0:
                motor.setPower(-0.5);
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
                motor.setPower(power);

                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

}
