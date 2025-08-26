package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Drivetrain {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    public void init (HardwareMap hardwareMap, boolean wheels) {
        // Declare motors
        if (wheels) {
            frontLeftMotor = hardwareMap.dcMotor.get("FL");
            backLeftMotor = hardwareMap.dcMotor.get("BL");
            frontRightMotor = hardwareMap.dcMotor.get("FR");
            backRightMotor = hardwareMap.dcMotor.get("BR");

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

    }

    public void move(double x, double y, double rx, float time) {
        ElapsedTime runtime = new ElapsedTime();

            double frontLeftPower = Math.min(y + x + rx, 1);
            double backLeftPower = Math.min(y - x + rx, 1);
            double frontRightPower = Math.min(y - x - rx, 1);
            double backRightPower = Math.min(y + x - rx, 1);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            if(runtime.seconds() < time) {}
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);


    }


}
