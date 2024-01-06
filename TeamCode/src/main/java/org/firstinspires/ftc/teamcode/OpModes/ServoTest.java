package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

@TeleOp(name = "servoTest")
@Config
public class ServoTest extends LinearOpMode {
    public static double position = 0;
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);


        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub h = new ExpansionHub(hardwareMap);

        Grippers leftClaw = new Grippers(new AutoServo(SERVO_PORTS.S5, true, true, 0, AutoServo.type.MICRO_SERVO),
                hardwareMap.get(DigitalChannel.class, "eD0"), telemetry);
        Grippers rightClaw = new Grippers(new AutoServo(SERVO_PORTS.S4, true, false, 0, AutoServo.type.MICRO_SERVO),
                hardwareMap.get(DigitalChannel.class, "eD1"), telemetry);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            leftClaw.update();
            rightClaw.update();
            if(gamepad1.a) rightClaw.drop();

            rightClaw.runTelemetry();
            telemetry.update();
        }
    }
}
