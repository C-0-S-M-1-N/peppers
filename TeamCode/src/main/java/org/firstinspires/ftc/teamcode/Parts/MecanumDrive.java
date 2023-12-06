package org.firstinspires.ftc.teamcode.Parts;

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
    private Gamepad gamepad1;

    public MecanumDrive(HardwareMap hardwareMap) {
        motorFrontLeft = hardwareMap.get(DcMotor.class ,"MotorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class ,"MotorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class ,"MotorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class ,"MotorBackRight");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void SetPower(double MFLPower, double MFRPower, double MBLPower, double MBRPower) {
        double power = 1f;
        power = Math.max(power, Math.abs(MFLPower));
        power = Math.max(power, Math.abs(MFRPower));
        power = Math.max(power, Math.abs(MBRPower));
        power = Math.max(power, Math.abs(MBLPower));

        motorFrontLeft.setPower(MFLPower / power);
        motorBackLeft.setPower(MBLPower / power);
        motorBackRight.setPower(MBRPower / power);
        motorFrontRight.setPower(MFRPower / power);
    }

    public void mergi(double fata, double roteste, double parte){
        double MFLPower, MFRPower, MBLPower, MBRPower;
        MFLPower = fata + roteste + parte;
        MFRPower = fata - roteste - parte;
        MBLPower = fata + roteste - parte;
        MBRPower = fata - roteste + parte;

        SetPower(MFLPower, MFRPower, MBLPower, MBRPower);
    }


}
//gamepad.left_stick_x
//gamepad.left_stick_y
//gamepad.left_trigger
//gamepad.right_trigger