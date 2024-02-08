package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.OutTakeExtension;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;

@TeleOp(name = "elevator tune")
@Config
public class ElevatorTuning extends LinearOpMode {
    public static Elevator elevator;
    public static ElevatorArm arm;
    public static double pos = 0;


    @Override
    public void runOpMode(){

        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap);
        FtcDashboard a = FtcDashboard.getInstance();
        Controls c = new Controls(gamepad1, gamepad2);
        telemetry = new MultipleTelemetry(telemetry, a.getTelemetry());
        ControlHub.telemetry = telemetry;

        elevator = new Elevator();
        arm = new ElevatorArm();
        ControlHub.setServoPosition(SERVO_PORTS.S4, 0);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            if(Controls.ElevatorUp){
                pos += 100;
                elevator.setTargetPosition(pos);
            }
            if(Controls.ElevatorDown){
                pos -= 100;
                elevator.setTargetPosition(pos);
            }
            elevator.update();
            arm.update();

            elevator.runTelemetry();
        }


    }
}
