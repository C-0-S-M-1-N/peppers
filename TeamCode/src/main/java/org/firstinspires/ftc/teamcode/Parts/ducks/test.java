package org.firstinspires.ftc.teamcode.Parts.ducks;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class test {
    private DcMotor motor1, motor2, motor3, motor4;
    public test(HardwareMap hardwareMap, Telemetry tel, String M1, String M2, String M3, String M4){
        motor1 = hardwareMap.get(DcMotor.class, M1);
        motor2 = hardwareMap.get(DcMotor.class, M2);
        motor3 = hardwareMap.get(DcMotor.class, M3);
        motor4 = hardwareMap.get(DcMotor.class, M4);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPawa(double p1, double p2, double p3, double p4){
        motor1.setPower(p1);
        motor2.setPower(p2);
        motor3.setPower(p3);
        motor4.setPower(p4);
    }
}
