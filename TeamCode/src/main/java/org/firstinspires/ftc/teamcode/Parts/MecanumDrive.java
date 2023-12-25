package org.firstinspires.ftc.teamcode.Parts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Part;

public class MecanumDrive{
    public static boolean Disable = false;
    private DcMotorEx LFmotor, LBmotor,
                      RFmotor, RBmotor;
    private Telemetry telemetry;
    public MecanumDrive(HardwareMap hm, Telemetry tele){
        telemetry = tele;
        LFmotor = hm.get(DcMotorEx.class, "LFmotor");
        LBmotor = hm.get(DcMotorEx.class, "LBmotor");
        RFmotor = hm.get(DcMotorEx.class, "RFmotor");
        RBmotor = hm.get(DcMotorEx.class, "RBmotor");
        MotorConfigurationType mct;

        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mct = LFmotor.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        LFmotor.setMotorType(mct);

        mct = LBmotor.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        LBmotor.setMotorType(mct);

        mct = RFmotor.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        RFmotor.setMotorType(mct);

        mct = RBmotor.getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        RBmotor.setMotorType(mct);

        LBmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LFmotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void update(double left_stick_y, double left_stick_x,
                       double right_trigger, double left_trigger){
        if(Disable) return;
        double rotation = right_trigger - left_trigger;
        double denominator = Math.max(left_stick_x + left_stick_y + rotation, 1);

        LFmotor.setPower((left_stick_x + left_stick_y + rotation)/denominator);
        LBmotor.setPower((left_stick_x - left_stick_y + rotation)/denominator);
        RFmotor.setPower((left_stick_x - left_stick_y - rotation)/denominator);
        RBmotor.setPower((left_stick_x + left_stick_y - rotation)/denominator);

    }

    public void runTelemetry(){
    }

}
