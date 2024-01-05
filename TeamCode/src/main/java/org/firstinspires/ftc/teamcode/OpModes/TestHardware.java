package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

@Config
@TeleOp(name = "testHardware")
public class TestHardware extends LinearOpMode {

    public static ElevatorArm arm;
    public static double pos = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub expansionHub = new ExpansionHub(hardwareMap);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        arm = new ElevatorArm(telemetry);
        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            arm.setPosition(pos);
            arm.update();
        }

    }
}
