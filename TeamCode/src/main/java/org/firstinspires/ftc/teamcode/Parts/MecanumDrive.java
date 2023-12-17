package org.firstinspires.ftc.teamcode.Parts;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MecanumDrive {
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
   public boolean speedDown;
   public  double x;
   private Telemetry telemetry;

    public MecanumDrive(HardwareMap hardwareMap, Telemetry tel) {
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

    public void SetPower(double MFLPower, double MFRPower, double MBLPower, double MBRPower) {
        double power;
        if(gamepad1.a) {
            speedDown = true;
            telemetry.addData("Nitro:", true);
        }else {
            speedDown = false;
            telemetry.addData("Nitro:", false);
        }
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

    public void mergi(double speed, double rotaion, double strafe){
        double MFLPower, MFRPower, MBLPower, MBRPower;

        MFLPower = speed + rotaion + strafe;
        MFRPower = speed - rotaion - strafe;
        MBLPower = speed + rotaion - strafe;
        MBRPower = speed - rotaion + strafe;

        telemetry.addData("Front Left Motor: ", MFLPower);
        telemetry.addData("Front Right Motor: ", MFRPower);
        telemetry.addData("Back Left Motor: ", MBLPower);
        telemetry.addData("Back Right Motor: ", MBRPower);

        SetPower(MFLPower, MFRPower, MBLPower, MBRPower);

    }


}
