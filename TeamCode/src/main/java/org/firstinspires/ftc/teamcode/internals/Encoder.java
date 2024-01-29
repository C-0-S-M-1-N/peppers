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
    private double prevPos, currentPos;
    public Encoder(DcMotorEx m){
        motor = m; direction = Direction.FORWARD;
    }
    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setDirection(Direction dir){
        direction = dir;
    }

    public double getPosition(){
        prevPos = currentPos;
        currentPos = motor.getCurrentPosition();
        return direction == Direction.FORWARD ? currentPos : -currentPos;
    }
    public double getVelocity(){
        return (currentPos - prevPos);
    }
}
