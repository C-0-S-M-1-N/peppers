package org.firstinspires.ftc.teamcode.internals;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;

public class ControlHub {
    private static DcMotorEx motor0, motor1, motor2, motor3;
    private static Encoder encoder0, encoder1, encoder2, encoder3;
    private static Servo servo0, servo1, servo2, servo3, servo4, servo5;

    private static void setMotorsToMax(){
        MotorConfigurationType mct = motor0.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor0.setMotorType(mct);

        mct = motor1.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor1.setMotorType(mct);

        mct = motor2.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor2.setMotorType(mct);

        mct = motor3.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor3.setMotorType(mct);

        resetEncoder(ENCODER_PORTS.E0);
        resetEncoder(ENCODER_PORTS.E1);
        resetEncoder(ENCODER_PORTS.E2);
        resetEncoder(ENCODER_PORTS.E3);

    }

    public ControlHub(HardwareMap hm){
        motor0 = hm.get(DcMotorEx.class, "cM0");
        motor1 = hm.get(DcMotorEx.class, "cM1");
        motor2 = hm.get(DcMotorEx.class, "cM2");
        motor3 = hm.get(DcMotorEx.class, "cM3");

        servo0 = hm.get(Servo.class, "cS0");
        servo1 = hm.get(Servo.class, "cS1");
        servo2 = hm.get(Servo.class, "cS2");
        servo3 = hm.get(Servo.class, "cS3");
        servo4 = hm.get(Servo.class, "cS4");
        servo5 = hm.get(Servo.class, "cS5");

        encoder0 = new Encoder(motor0);
        encoder1 = new Encoder(motor1);
        encoder2 = new Encoder(motor2);
        encoder3 = new Encoder(motor3);

        setMotorsToMax();

    }

    public static double getMotorPosition(ENCODER_PORTS encoder){
        switch(encoder){
            case E0:
                return motor0.getCurrentPosition();
            case E1:
                return motor1.getCurrentPosition();
            case E2:
                return motor2.getCurrentPosition();
            case E3:
                return motor3.getCurrentPosition();
        }
        return 0;
    }
    public static double getMotorVelocity(ENCODER_PORTS encoder){
        switch (encoder){
            case E0:
                return motor0.getVelocity();
            case E1:
                return motor1.getVelocity();
            case E2:
                return motor2.getVelocity();
            case E3:
                return motor3.getVelocity();
        }
        return 0;
    }
    public static void setServoPosition(SERVO_PORTS servo, double position){
        switch (servo) {
            case S0:
                servo0.setPosition(position);
                break;
            case S1:
                servo1.setPosition(position);
                break;
            case S2:
                servo2.setPosition(position);
                break;
            case S3:
                servo3.setPosition(position);
                break;
            case S4:
                servo4.setPosition(position);
                break;
            case S5:
                servo5.setPosition(position);
                break;
        }
    }
    public static double getCurrentFromMotor(MOTOR_PORTS motor, CurrentUnit unit){
        switch (motor){
            case M0:
                return motor0.getCurrent(unit);
            case M1:
                return motor1.getCurrent(unit);
            case M2:
                return motor2.getCurrent(unit);
            case M3:
                return motor3.getCurrent(unit);
        }
        return 0;
    }
    public static void setServoDirection(SERVO_PORTS servo, Servo.Direction dir) {
        switch (servo) {
            case S0:
                servo0.setDirection(dir);
                break;
            case S1:
                servo1.setDirection(dir);
                break;
            case S2:
                servo2.setDirection(dir);
                break;
            case S3:
                servo3.setDirection(dir);
                break;
            case S4:
                servo4.setDirection(dir);
                break;
            case S5:
                servo5.setDirection(dir);
                break;
        }
    }
    public static void setMotorDirection(MOTOR_PORTS motor, DcMotorSimple.Direction dir){
        switch (motor){
            case M0:
                motor0.setDirection(dir);
                break;
            case M1:
                motor1.setDirection(dir);
                break;
            case M2:
                motor2.setDirection(dir);
                break;
            case M3:
                motor3.setDirection(dir);
                break;
        }
    }
    public static void setMotorPower(MOTOR_PORTS motor, double power){
        switch (motor){
            case M0:
                motor0.setPower(power);
                break;
            case M1:
                motor1.setPower(power);
                break;
            case M2:
                motor2.setPower(power);
                break;
            case M3:
                motor3.setPower(power);
                break;
        }
    }
    public void setEncoderDirection(ENCODER_PORTS encoder, Encoder.Direction dir){
        switch (encoder){
            case E0:
                encoder0.setDirection(dir);
                break;
            case E1:
                encoder1.setDirection(dir);
                break;
            case E2:
                encoder2.setDirection(dir);
                break;
            case E3:
                encoder3.setDirection(dir);
                break;
        }
    }
    public static void resetEncoder(ENCODER_PORTS encoder){
        switch (encoder){
            case E0:
                encoder0.reset();
                break;
            case E1:
                encoder1.reset();
                break;
            case E2:
                encoder2.reset();
                break;
            case E3:
                encoder3.reset();
                break;
        }
    }


}
