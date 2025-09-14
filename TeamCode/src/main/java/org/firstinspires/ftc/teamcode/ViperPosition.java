package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Viperslides with movement of robot")
public class ViperPosition extends LinearOpMode {

    double slidestargetposition = 0;
    double lefttrigger = 0;
    double righttrigger = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor Slides = hardwareMap.dcMotor.get("slides");
        Slides.setDirection(DcMotorSimple.Direction.REVERSE);
        Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DcMotor FL = hardwareMap.dcMotor.get("FL");
        DcMotor BL = hardwareMap.dcMotor.get("BL");
        DcMotor FR = hardwareMap.dcMotor.get("FR");
        DcMotor BR = hardwareMap.dcMotor.get("BR");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        Slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Resets encoder for driving
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // sets mode to run without encoder because we don't need to caluclate volocity
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide slide = new Slide();
        telemetry.addData("Robot is ready!", "Skibidi Toliet Rizz!");
        telemetry.update();
        waitForStart();

        double leftstick = gamepad1.left_stick_y;
        while (opModeIsActive()) {
            lefttrigger = gamepad1.left_trigger;
            righttrigger = gamepad1.right_trigger;
            Slides.setPower((-lefttrigger)+righttrigger);
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FLPower = (y + x + rx) / denominator;
            double BLPower = (y - x + rx) / denominator;
            double FRPower = (y - x - rx) / denominator;
            double BRPower = (y + x - rx) / denominator;

            FL.setPower(FLPower);
            BL.setPower(BLPower);
            FR.setPower(FRPower);
            BR.setPower(BRPower);
            telemetry.addData("Current position", Slides.getCurrentPosition());
            telemetry.addData("Stick position", leftstick);
            telemetry.addData("Left Trigger", lefttrigger);
            telemetry.addData("Right Trigger", righttrigger);
            telemetry.update();
        }
    }
}

