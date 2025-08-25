package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Autonomous
public class AutonomousTest extends LinearOpMode{

    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain();
        drive.init(hardwareMap, true);
        drive.move(0, 0.5, 0, 2);
    }
}
