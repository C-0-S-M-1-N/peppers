package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "testHardware")
public class TestHardware extends LinearOpMode {
    public DcMotorEx EncoderMotor, RunMotor;
    public Servo servo;

    public boolean aPress, bPress, xPress;
    public static int targetPosition = 0;
    public static double servoPosition = 0;
    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = FtcDashboard.getInstance().getTelemetry();

       EncoderMotor = hardwareMap.get(DcMotorEx.class, "chm0");
       RunMotor = hardwareMap.get(DcMotorEx.class, "chm1");
       servo = hardwareMap.get(Servo.class, "chs5");

       RunMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       EncoderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       EncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       EncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       EncoderMotor.setTargetPosition(0);
       EncoderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){

            RunMotor.setPower(power);
            EncoderMotor.setTargetPosition(targetPosition);
            servo.setPosition(servoPosition);

            EncoderMotor.setPower(1);
            telemetry.addData("EncoderMotor", EncoderMotor.getCurrentPosition());
            telemetry.addData("EncoderMotor current used", EncoderMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("RunMotor current used", RunMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();

            aPress = gamepad1.a;
            bPress = gamepad1.b;
            xPress = gamepad1.x;
        }

    }
}
