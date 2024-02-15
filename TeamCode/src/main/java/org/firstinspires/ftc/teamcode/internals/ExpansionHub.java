package org.firstinspires.ftc.teamcode.internals;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Mutex;

import java.util.List;

public class ExpansionHub {
    public static DcMotorEx[] motor = new DcMotorEx[4];
    public static Encoder[] encoder = new Encoder[4];
    public static Servo[] servo = new Servo[6];
    private static double[] servo_cache = new double[6], motor_cache = new double[4];

    public static double voltage;
    public static final double compensation = 12;
    public static IMU imu;
    public static DistanceSensor sensor;

    private static void setMotorsToMax(){
        MotorConfigurationType mct = motor[0].getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor[0].setMotorType(mct);

        mct = motor[1].getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor[1].setMotorType(mct);

        mct = motor[1].getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor[2].setMotorType(mct);

        mct = motor[3].getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor[3].setMotorType(mct);

        resetEncoder(ENCODER_PORTS.E0);
        resetEncoder(ENCODER_PORTS.E1);
        resetEncoder(ENCODER_PORTS.E2);
        resetEncoder(ENCODER_PORTS.E3);

    }
    public static Thread runI2Cdevices;
    public static Mutex I2CMutex = new Mutex();
    public static double ImuYawAngle = 0, sensorDistance = 0;

    public ExpansionHub(HardwareMap hm){
        motor[0] = hm.get(DcMotorEx.class, "eM0");
        motor[1] = hm.get(DcMotorEx.class, "eM1");
        motor[2] = hm.get(DcMotorEx.class, "eM2");
        motor[3] = hm.get(DcMotorEx.class, "eM3");

        for(int i = 0; i < 4; i++){
            motor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        for(int i = 0; i < 6; i++){
            servo[i] = hm.get(Servo.class, "eS" + (i + '0'));
        }
        for(int i = 0; i < 4; i++){
            encoder[i] = new Encoder(motor[i]);
        }

        voltage = hm.voltageSensor.iterator().next().getVoltage();
        imu = hm.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));
        imu.resetYaw();

        sensor = hm.get(DistanceSensor.class, "sensor");

        runI2Cdevices = new Thread(() -> {
            while(!I2CMutex.kill) {
                double x = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double y = sensor.getDistance(DistanceUnit.MM);

                I2CMutex.lock();
                ImuYawAngle = x;
                sensorDistance = y;
                I2CMutex.unlock();
            }
        });

        setMotorsToMax();

    }
    private static double angle, dist;
    public static void update(){
        angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        dist = sensor.getDistance(DistanceUnit.MM);

    }

    public static double getRobotRotation(AngleUnit unit){
        return unit == AngleUnit.DEGREES ? Math.toDegrees(angle) : angle;
    }
    public static double getSensorValue(){
        return dist;
    }

    public static double getEncoderPosition(ENCODER_PORTS encoder_port){
        switch(encoder_port){
            case E0:
                return encoder[0].getPosition();
            case E1:
                return encoder[1].getPosition();
            case E2:
                return encoder[2].getPosition();
            case E3:
                return encoder[3].getPosition();
        }
        return 0;
    }
    public static double getMotorVelocity(ENCODER_PORTS encoder){
        switch (encoder){
            case E0:
                return motor[0].getVelocity();
            case E1:
                return motor[1].getVelocity();
            case E2:
                return motor[2].getVelocity();
            case E3:
                return motor[3].getVelocity();
        }
        return 0;
    }
    public static void setServoPosition(SERVO_PORTS port, double position){
        switch (port) {
            case S0:
                if(servo_cache[0] != position) {
                    servo[0].setPosition(position);
                    servo_cache[0] = position;
                }
                break;
            case S1:
                if(servo_cache[1] != position) {
                    servo[1].setPosition(position);
                    servo_cache[1] = position;
                }
                break;
            case S2:
                if(servo_cache[2] != position) {
                    servo[2].setPosition(position);
                    servo_cache[2] = position;
                }
                break;
            case S3:
                if(servo_cache[3] != position) {
                    servo[3].setPosition(position);
                    servo_cache[3] = position;
                }
                break;
            case S4:
                if(servo_cache[4] != position) {
                    servo[4].setPosition(position);
                    servo_cache[4] = position;
                }
                break;
            case S5:
                if(servo_cache[5] != position) {
                    servo[5].setPosition(position);
                    servo_cache[5] = position;
                }
                break;
        }
    }
    public static double getCurrentFromMotor(MOTOR_PORTS port, CurrentUnit unit){
        switch (port){
            case M0:
                return motor[0].getCurrent(unit);
            case M1:
                return motor[1].getCurrent(unit);
            case M2:
                return motor[2].getCurrent(unit);
            case M3:
                return motor[3].getCurrent(unit);
        }
        return 0;
    }
    public static void setServoDirection(SERVO_PORTS port, Servo.Direction dir) {
        switch (port) {
            case S0:
                servo[0].setDirection(dir);
                break;
            case S1:
                servo[1].setDirection(dir);
                break;
            case S2:
                servo[2].setDirection(dir);
                break;
            case S3:
                servo[3].setDirection(dir);
                break;
            case S4:
                servo[4].setDirection(dir);
                break;
            case S5:
                servo[5].setDirection(dir);
                break;
        }
    }
    public static void setMotorDirection(MOTOR_PORTS port, DcMotorSimple.Direction dir){
        switch (port){
            case M0:
                motor[0].setDirection(dir);
                break;
            case M1:
                motor[1].setDirection(dir);
                break;
            case M2:
                motor[2].setDirection(dir);
                break;
            case M3:
                motor[3].setDirection(dir);
                break;
        }
    }
    public static void setMotorPower(MOTOR_PORTS port, double power){
        switch (port){
            case M0:
                if(motor_cache[0] != power){
                    motor[0].setPower(power * compensation / voltage);
                    motor_cache[0] = power;
                }
                break;
            case M1:
                if(motor_cache[1] != power){
                    motor[1].setPower(power * compensation / voltage);
                    motor_cache[1] = power;
                }
                break;
            case M2:
                if(motor_cache[2] != power){
                    motor[2].setPower(power * compensation / voltage);
                    motor_cache[2] = power;
                }
                break;
            case M3:
                if(motor_cache[3] != power){
                    motor[3].setPower(power * compensation / voltage);
                    motor_cache[3] = power;
                }
                break;
        }
    }
    public void setEncoderDirection(ENCODER_PORTS port, Encoder.Direction dir){
        switch (port){
            case E0:
                encoder[0].setDirection(dir);
                break;
            case E1:
                encoder[1].setDirection(dir);
                break;
            case E2:
                encoder[2].setDirection(dir);
                break;
            case E3:
                encoder[3].setDirection(dir);
                break;
        }
    }
    public static void resetEncoder(ENCODER_PORTS port){
        switch (port){
            case E0:
                encoder[0].reset();
                break;
            case E1:
                encoder[1].reset();
                break;
            case E2:
                encoder[2].reset();
                break;
            case E3:
                encoder[3].reset();
                break;
        }
    }

    public static void teleMotorCurrents(Telemetry telemetry) {

        telemetry.addData("EM0:", getCurrentFromMotor(MOTOR_PORTS.M0, CurrentUnit.AMPS));
        telemetry.addData("EM1:", getCurrentFromMotor(MOTOR_PORTS.M1, CurrentUnit.AMPS));
        telemetry.addData("EM2:", getCurrentFromMotor(MOTOR_PORTS.M2, CurrentUnit.AMPS));
        telemetry.addData("EM3:", getCurrentFromMotor(MOTOR_PORTS.M3, CurrentUnit.AMPS));
        telemetry.addData("Expansion Power:", getCurrentFromMotor(MOTOR_PORTS.M0, CurrentUnit.AMPS) + getCurrentFromMotor(MOTOR_PORTS.M1, CurrentUnit.AMPS) + getCurrentFromMotor(MOTOR_PORTS.M2, CurrentUnit.AMPS) + getCurrentFromMotor(MOTOR_PORTS.M3, CurrentUnit.AMPS));

    }


}