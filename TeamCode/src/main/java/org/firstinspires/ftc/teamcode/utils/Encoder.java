package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Encoder {
    private DcMotorEx encoder;
    private double velocity, position;

    public Encoder(DcMotorEx motor, boolean reverse){
        encoder = motor;
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setDirection(reverse ?
                DcMotorSimple.Direction.REVERSE :
                DcMotorSimple.Direction.FORWARD);
    }

    public void update(){
        position = encoder.getCurrentPosition();
        velocity = encoder.getVelocity(AngleUnit.RADIANS);
    }

    public double getPosition(){
        return position;
    }

    /*
    * @brief returns the velocity in radians/s
    * */
    public double getVelocity(){
        return velocity;
    }

}
