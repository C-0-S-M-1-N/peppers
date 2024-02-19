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
import org.firstinspires.ftc.teamcode.Parts.MotionProfile;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

@TeleOp(name = "elevator tune")
@Config
public class ElevatorTuning extends LinearOpMode {
    public static Elevator elevator;
    public static double pos = 0, maxUp = 925;
    public static double lvl = maxUp / 11;
    public static boolean update = false;


    @Override
    public void runOpMode() throws InterruptedException {

        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap);
        FtcDashboard a = FtcDashboard.getInstance();
        Controls c = new Controls(gamepad1, gamepad2);
        telemetry = new MultipleTelemetry(telemetry, a.getTelemetry());
        ControlHub.telemetry = telemetry;

        elevator = new Elevator();
        ElevatorArm arm = new ElevatorArm();
        arm.setArmAngle(0);
        arm.setOrientation(0);
        arm.setPivotAngle(0);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            if(Controls.ElevatorUp || update){
                elevator.setTargetPosition(pos*lvl);
                update = false;
            }
            if(Controls.ElevatorDown || update){
                elevator.setTargetPosition(pos * lvl);
                update = false;
            }
            elevator.update();
            arm.update();

            elevator.update_values();
            arm.update_values();

            elevator.runTelemetry();
            c.loop();
        }


    }
}
