package org.firstinspires.ftc.teamcode.Parts;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MDTrig {
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    public boolean speedDown;
    public  double x;
    private Telemetry telemetry;

    public MDTrig(@NonNull HardwareMap hardwareMap, Telemetry tel) {
        x=1.7f;
        speedDown = false;
        motorFrontLeft = hardwareMap.get(DcMotor.class ,"mfl");
        motorFrontRight = hardwareMap.get(DcMotor.class ,"mfr");
        motorBackLeft = hardwareMap.get(DcMotor.class ,"mbl");
        motorBackRight = hardwareMap.get(DcMotor.class ,"mbr");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void SetPower(double phi, double power, double turn, boolean boost) {
        double MFLPower = 0, MFRPower = 0, MBLPower = 0, MBRPower = 0, sin, cos, max;
        x=1;

        if(boost){
            x = 1;
        }else {
            x = 1.7;
        }

        sin = Math.sin(phi - Math.PI/4);
        cos = Math.cos(phi - Math.PI/4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

            MFLPower = power * cos / max + turn;
            MFRPower = power * sin / max - turn;
            MBLPower = power * sin / max + turn;
            MBRPower = power * cos / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            MFLPower   /= power + Math.abs(turn);
            MFRPower /= power + Math.abs(turn);
            MBLPower    /= power + Math.abs(turn);
            MBRPower  /= power + Math.abs(turn);
        }

        motorFrontRight.setPower(MFRPower*x);
        motorFrontLeft.setPower(MFLPower*x);
        motorBackRight.setPower(MBRPower*x);
        motorBackLeft.setPower(MBLPower*x);

//        telemetry.addData("Front Left Motor: ", MFLPower);
//        telemetry.addData("Front Right Motor: ", MFRPower);
//        telemetry.addData("Back Left Motor: ", MBLPower);
//        telemetry.addData("Back Right Motor: ", MBRPower);
//        telemetry.update();
    }

    public void mergi(double speed, double rotation, double strafe, boolean boost){
        double phi, power;
        phi = Math.atan2(strafe, speed);
        power = Math.hypot(strafe, speed);

        SetPower(phi, power, rotation, boost);

    }


}
