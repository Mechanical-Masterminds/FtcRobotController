package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ViperslideTestAlex")
public class viperslidesalex extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor Slides = hardwareMap.dcMotor.get("slides");
        Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Robot is ready!", "Skibidi Toliet Rizz!");
        telemetry.update();
        waitForStart();

        double leftstick = gamepad1.left_stick_y;
        while (opModeIsActive()) {
            leftstick = -gamepad1.left_stick_y;
            if((Slides.getCurrentPosition() * -1) > 2910) {
                if((leftstick) > 0) {
                    leftstick = 0;
                }
                if((leftstick) < 0) {

                }
                if((leftstick) == 0) {}
            }
            if((Slides.getCurrentPosition()) == 0) {
                if((leftstick) < 0){}
                if((leftstick) < 0) {
                    leftstick = 0;
                }
                if((leftstick * -1) == 0) {}
            }
            if(leftstick >= 0.2) {
                leftstick = 0.2;
            }
            Slides.setPower(leftstick);
            telemetry.addData("Current position",(Slides.getCurrentPosition() * -1) );
            telemetry.addData("Motorpower", leftstick);
            telemetry.update();
        }
    }

    @com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "DemoTeleOp")

    public static class ViperSlidesOp extends LinearOpMode {

        private DcMotor slideMotor;
        private final double slidePowerUp = 0.5;
        private final double slidePowerDown = -0.5;
        private int position = 0;
        private int previousPosition=0;

        private final double minSlidePosition = 0;
        private final double maxSlidePosition = 100;

        @Override
        public void runOpMode() throws InterruptedException {
            slideMotor = hardwareMap.dcMotor.get("slide");
            slideMotor.setDirection(DcMotor.Direction.REVERSE);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Robot robot = new Robot();

            double leftstick = -gamepad1.left_stick_y;

            position += (int) leftstick * 50;
            position = Math.min(position, 2910);
            position = Math.max(position, 0);

            slideMotor.setTargetPosition(position);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (position-previousPosition > 0){
                slideMotor.setPower(leftstick);
            } else if (position-previousPosition < 0) {
                slideMotor.setPower(-leftstick);
            }

            previousPosition = position;


        }
    }
}