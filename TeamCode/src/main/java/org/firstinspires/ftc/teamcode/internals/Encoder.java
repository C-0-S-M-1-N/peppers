package org.firstinspires.ftc.teamcode.internals;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;

public class Encoder {
    public enum Direction{
        FORWARD,
        REVERSE
    }
    private Direction direction;
    private DcMotorEx motor;
    public Encoder(DcMotorEx m){
        motor = m;
    }
    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        direction = Direction.FORWARD;
    }
    public void setDirection(Direction dir){
        direction = dir;
    }

    public double getPosition(){
        double normalizePos = Math.abs(motor.getCurrentPosition());
        return direction == Direction.FORWARD ? normalizePos : -normalizePos;
    }
    public double getVelocity(){
        return Math.abs(motor.getVelocity());
    }
}
