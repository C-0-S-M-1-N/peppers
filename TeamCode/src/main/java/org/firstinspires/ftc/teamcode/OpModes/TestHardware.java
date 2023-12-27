package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

@Config
@TeleOp(name = "testHardware")
public class TestHardware extends LinearOpMode {
    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = FtcDashboard.getInstance().getTelemetry();
        ControlHub controlHub = new ControlHub(hardwareMap);

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            ControlHub.setMotorPower(MOTOR_PORTS.M0, power);

            telemetry.addData("pos", ControlHub.getMotorPosition(ENCODER_PORTS.E3));
            telemetry.update();
        }

    }
}
