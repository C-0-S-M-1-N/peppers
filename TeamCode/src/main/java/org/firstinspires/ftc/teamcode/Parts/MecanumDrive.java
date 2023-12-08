package org.firstinspires.ftc.teamcode.Parts;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class MecanumDrive {
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
   public boolean speedDown;
   public  double x;

    public MecanumDrive(HardwareMap hardwareMap) {
        x=1.5f;
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

    public void SetPower(double MFLPower, double MFRPower, double MBLPower, double MBRPower) {
        double power = 1f;
        if(speedDown)
            power = 1f;
            power = Math.max( power, Math.abs(MFLPower));
            power = Math.max( power, Math.abs(MFRPower));
            power = Math.max( power, Math.abs(MBRPower));
            power = Math.max( power, Math.abs(MBLPower));
            if(speedDown) {
                motorFrontLeft.setPower(MFLPower / power);
                motorBackLeft.setPower(MBLPower / power);
                motorBackRight.setPower(MBRPower / power);
                motorFrontRight.setPower(MFRPower / power);
            }else{
                motorFrontLeft.setPower(MFLPower / power / x);
                motorBackLeft.setPower(MBLPower / power / x);
                motorBackRight.setPower(MBRPower / power / x);
                motorFrontRight.setPower(MFRPower / power / x);
            }
    }

    public void mergi(double fata, double roteste, double parte){
        double MFLPower, MFRPower, MBLPower, MBRPower;
        MFLPower = fata + roteste + parte;
        MFRPower = fata - roteste - parte;
        MBLPower = fata + roteste - parte;
        MBRPower = fata - roteste + parte;

        SetPower(MFLPower, MFRPower, MBLPower, MBRPower);
    }


    public void putere(){
        if(gamepad1.a)
            speedDown = true;
        else
            speedDown = false;
    }

}
