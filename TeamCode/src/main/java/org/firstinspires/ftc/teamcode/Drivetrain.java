package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Drivetrain {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    public double leftfronterror = 0;
    public double leftfrontderivative = 0;
    public double leftfrontlasterror = 0;
    public double leftfrontintegralsum = 0;
    ElapsedTime timer = new ElapsedTime();




    //PID
    double Kp = 1;
    double Ki = 1;
    double Kd = 1;


    public void init (HardwareMap hardwareMap) {
        // Declare motors

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

    public void move(double x, double y, double rx) {

        double frontLeftPower = Math.min(y + x + rx, 1);
        double backLeftPower = Math.min(y - x + rx, 1);
        double frontRightPower = Math.min(y - x - rx, 1);
        double backRightPower = Math.min(y + x - rx, 1);

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);


    }

    private int connvertFeetToTicks(double feet){
        int ticks = (int) (feet* 183.4671916f);
        return ticks;
    }


    public void moveToPosition(double x){
        int reference = connvertFeetToTicks(x);
        // left front
        ElapsedTime timer = new ElapsedTime();

        double lastError=0;
        double integralSum=0;
        while (frontLeftMotor.getCurrentPosition() != reference) {

            double encoderPosition = frontLeftMotor.getCurrentPosition();
            double error = reference - encoderPosition;
            double derivative = (error - lastError) / timer.seconds();
            integralSum += (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            //we control the x, y, and theta values, and we move the motors based on those values
            //we don't have to do PID for each motor. I might be wrong
            //https://www.ctrlaltftc.com/practical-examples/drivetrain-control

            if(x > 0) {
                frontLeftMotor.setPower(out);
                frontRightMotor.setPower(-out);
                backLeftMotor.setPower(-out);
                backRightMotor.setPower(out);
            } else {
                frontLeftMotor.setPower(-out);
                frontRightMotor.setPower(out);
                backLeftMotor.setPower(out);
                backRightMotor.setPower(-out);
            }

            lastError = error;

            // reset the timer for next time
            timer.reset();

        }

    }






}
