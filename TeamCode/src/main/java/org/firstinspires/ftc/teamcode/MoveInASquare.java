/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
//import com.pedropathing.

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class MoveInASquare extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        waitForStart();
        Follower follower =  Constants.createFollower(hardwareMap,hardwareMap);
        
        while(opModeIsActive() && !isStopRequested()) {
            path = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup1Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                    .addPath(new BezierLine(pickup1Pose, scorePose))
                    .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                    .build();
            follower.followPath(path);
        }

    }
}
*/