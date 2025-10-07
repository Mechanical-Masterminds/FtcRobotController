package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
@TeleOp
public class TurretStateMachine extends LinearOpMode{
    public enum TurretState {
        Obilisk_April_Tag,
        Red_Goal_April_Tag,
        Blue_Goal_April_Tag,
        Start_State,
        Red_Goal_Red_Alliance,
        Red_Goal_Blue_Alliance,
        Red_Goal_Red_Alliance_OffCenter,
        Red_Goal_Blue_Alliance_Rotate,
        Obilisk_April_Tag_Unkown_Value,
        Blue_Goal_Blue_Alliance,
        Blue_Goal_Red_Alliance,
        Blue_Goal_Red_Alliance_Rotate,
        Blue_Goal_Blue_Alliance_OffCenter,


    };
    TurretState turretState = TurretState.Start_State;
    private Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // get the pipeline for limelight
        limelight.setPollRateHz(100);
        telemetry.addData("The robot", "is ready!");
        telemetry.update();
        waitForStart();
        if(isStopRequested()) return;
        while(opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            switch (turretState) {
                case Start_State:
                    if (result != null && result.isValid()) {
                       double id = = result.t
                    } else {
                        break;
                    }
                    break;
                case Obilisk_April_Tag:
                    if (Math.abs(liftMotor.getCurrentPosition() - LIFT_HIGH) < 10) {
                        // our threshold is within
                        // 10 encoder ticks of our target.
                        // this is pretty arbitrary, and would have to be
                        // tweaked for each robot.

                        // set the lift dump to dump
                        liftDump.setTargetPosition(DUMP_DEPOSIT);

                        liftTimer.reset();
                        liftState = LiftState.LIFT_DUMP;
                    }
                    break;
                case LIFT_DUMP:
                    if (liftTimer.seconds() >= DUMP_TIME) {
                        // The robot waited long enough, time to start
                        // retracting the lift
                        liftDump.setTargetPosition(DUMP_IDLE);
                        liftMotor.setTargetPosition(LIFT_LOW);
                        liftState = LiftState.LIFT_RETRACT;
                    }
                    break;
                case LIFT_RETRACT:
                    if (Math.abs(liftMotor.getCurrentPosition() - LIFT_LOW) < 10) {
                        liftState = LiftState.LIFT_START;
                    }
                    break;
                default:
                    // should never be reached, as liftState should never be null
                    liftState = LiftState.LIFT_START;
            }
        }
    }
}
