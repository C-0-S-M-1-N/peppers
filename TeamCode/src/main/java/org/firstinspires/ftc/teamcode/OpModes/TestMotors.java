package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

@TeleOp(name = "TestMotors")
@Config
public class TestMotors extends LinearOpMode {
    public static MOTOR_PORTS port = MOTOR_PORTS.M3;
    public static double power = 0;
    public static DcMotorSimple.Direction dir = DcMotorSimple.Direction.FORWARD;
    public static Hubs hub = Hubs.CONTROL_HUB;

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            if(hub == Hubs.CONTROL_HUB) {
                ControlHub.setMotorDirection(port, dir);
                ControlHub.setMotorPower(port, power);
            } else {
                ExpansionHub.setMotorDirection(port, dir);
                ExpansionHub.setMotorPower(port, power);
            }
            telemetry.addData("power consumption", ControlHub.getCurrentFromMotor(port, CurrentUnit.MILLIAMPS));
            telemetry.update();
        }
    }

}
