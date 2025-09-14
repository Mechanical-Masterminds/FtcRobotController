package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeBootWheel extends LinearOpMode{
    private DcMotor intakeMotor;
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.dcMotor.get("INTAKE");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double rightStickX = gamepad1.right_stick_x;
            intakeMotor.setPower(Math.min(Math.max(0,rightStickX),1));
            telemetry.addData("Power of intake:", intakeMotor.getPower());
            telemetry.update();
        }
    }
}
