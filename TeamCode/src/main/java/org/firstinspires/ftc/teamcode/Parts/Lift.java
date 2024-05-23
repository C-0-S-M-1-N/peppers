package org.firstinspires.ftc.teamcode.Parts;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.lang.reflect.Array;

@Config
public class Lift {
    private DcMotor motorLift1, motorLift2, motorLift3;
    private PIDController pid;


    private enum lift_positions{
        TOP,
        MIDDLE,
        RESET
    }
    public static lift_positions lift;
    public static int speed, trgPosition;


    private Telemetry telemetry;
    public void setPidCoefficients(double kp, double ki, double kd){
        pid.setPidCoefficients(new PIDCoefficients(kp, ki, kd));
    }

    public Lift(HardwareMap hardwareMap, Telemetry tel, double kp, double ki, double kd){
        motorLift1 = hardwareMap.get(DcMotor.class, "lift1");
        motorLift2 = hardwareMap.get(DcMotor.class, "lift2");
        motorLift3 = hardwareMap.get(DcMotor.class, "lift3");

        motorLift1.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLift2.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = tel;

        DcMotor[] motors = {motorLift1, motorLift2, motorLift3};
        for (DcMotor moto : motors) {
            MotorConfigurationType mct = moto.getMotorType();
            mct.setAchieveableMaxRPMFraction(1.0);
            moto.setMotorType(mct);
            moto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            moto.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        pid = new PIDController(kp, ki, kd);
        pid.setMaxActuatorOutput(1.0);
    }

    public void initiation(){
        lift = lift_positions.RESET;
    }
    public void prototip(int trgPosition_, boolean isButtonPressed){
        double power;
        telemetry.addData("current position: ", motorLift1.getCurrentPosition());
        telemetry.addData("target position: ", pid.getTargetPosition());
        if(isButtonPressed)pid.setTargetPosition(trgPosition_);
        power = pid.calculatePower(motorLift1.getCurrentPosition());
        motorLift1.setPower(power);
        motorLift2.setPower(power);
        motorLift3.setPower(power);
    }
   public void update(boolean d_up, boolean d_down){
        double power;
        lift_positions liftcur = lift_positions.RESET;
        if(d_up && lift == lift_positions.RESET)
            lift = lift_positions.MIDDLE;
        else if (d_up && lift == lift_positions.MIDDLE)
            lift = lift_positions.TOP;
        else if(d_down && lift == lift_positions.TOP)
            lift = lift_positions.MIDDLE;
        else if(d_down && lift == lift_positions.MIDDLE)
            lift = lift_positions.RESET;

        if(liftcur != lift)
        switch (lift) {
            case RESET:
                pid.setTargetPosition(0);
                liftcur = lift_positions.RESET;
                break;
            case MIDDLE:
                liftcur = lift_positions.MIDDLE;
                pid.setTargetPosition(650);
                break;
            case TOP:
                liftcur = lift_positions.TOP;
                pid.setTargetPosition(1100);
                break;
       }
      power = pid.calculatePower(motorLift1.getCurrentPosition());
        motorLift1.setPower(power);
        motorLift2.setPower(power);
        motorLift3.setPower(power);
    }

    private void setTargetPositionToMotors(int val){
        motorLift1.setTargetPosition(val);
        motorLift2.setTargetPosition(val);
        motorLift3.setTargetPosition(val);
    }
}
