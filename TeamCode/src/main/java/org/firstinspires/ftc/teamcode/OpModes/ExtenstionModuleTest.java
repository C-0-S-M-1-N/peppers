package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Components.OutTakeExtensionModule;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;

@TeleOp(name = "OutTakeExtensionModule", group = "tune")
@Config
public class ExtenstionModuleTest extends LinearOpMode {
    OutTakeExtensionModule module;
    public static boolean extend = false;
    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, null);
        module = new OutTakeExtensionModule();

        waitForStart();

        while (opModeIsActive()){
            if(extend && module.isRetracted()) module.extend();
            if(!extend && module.isExtended()) module.retract();
        }
    }
}
