package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.OutTakeExtension;
import org.firstinspires.ftc.teamcode.Parts.MotionProfile;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import java.util.ArrayList;

@TeleOp(name = "elevator tune")
@Config
public class ElevatorTuning extends LinearOpMode {
    public static Elevator elevator;
    public static double pos = 0, maxUp = 1365;
    public static double lvl = maxUp / 11;
    public static boolean update = false;
    public static boolean climb = false;


    @Override
    public void runOpMode() throws InterruptedException {

        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap, new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>()));
        Controls c = new Controls(gamepad1, gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ControlHub.telemetry = telemetry;

        elevator = new Elevator();
        ElevatorArm arm = new ElevatorArm();
        arm.setArmAngle(0);
        arm.setOrientation(0);
        arm.setPivotAngle(0);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){

            for(LynxModule m : ControlHub.all){
                m.clearBulkCache();
            }
            if(climb){
                for(int i = 0; i < 3; i++){
                    ControlHub.motor[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    ControlHub.motor[i].setPower(-1);
                }
                if(ControlHub.motor[0].getCurrentPosition() <= 50){
                    climb = false;
                }
                continue;
            }

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

            telemetry.update();
        }




    }
}
