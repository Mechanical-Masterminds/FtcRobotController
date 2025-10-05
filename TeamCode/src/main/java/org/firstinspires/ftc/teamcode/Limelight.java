package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class Limelight extends OpMode {
    private Limelight3A limelight; //any camera here
    private Follower follower;
    private boolean following = false;
    private final Pose TARGET_LOCATION = new Pose(); //Put the target location here
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose()); //set your starting pose
    }
    @Override
    public void start() {
        limelight.start();
    }
    @Override
    public void loop() {
        follower.update();
        //if you're not using limelight you can follow the same steps: build an offset pose, put your heading offset, and generate a path etc
        if (!following) {
            follower.followPath(
                    follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
                            .setLinearHeadingInterpolation(follower.getHeading(), TARGET_LOCATION.minus(follower.getPose()).getAsVector().getTheta())
                            .build()
            );
        }
        //This uses the aprilTag to relocalize your robot
        //You can also create a custom AprilTag fusion Localizer for the follower if you want to use this by default for all your autos
        setRobotPoseFromCamera();
        if (following && !follower.isBusy()) following = false;
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
    }

    private void setRobotPoseFromCamera() {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                double heading = botpose.getOrientation().getYaw();
                follower.setPose(new Pose(x, y, heading, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE));
            }
        }




    }
}
//this was commented for testing
/*@TeleOp
public class Limelight extends OpMode {
    public static Limelight3A camera; //any camera here
    public static Follower follower;
    public static boolean following = false;
    public static Pose TARGET_LOCATION = new Pose(); //Put the target location here

    @Override
    public void init() {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose()); //set your starting pose
    }

    @Override
    public void start() {
        camera.start();
    }

    @Override
    public void loop() {
        follower.update();


        if (!following){
            follower.getPose();
        }else{
            follower.setPose(getRobotPoseFromCamera());
        }
        telemetry.addData("Position", getRobotPoseFromCamera().getX());
        //This uses the aprilTag to relocalize your robot
        //You can also create a custom AprilTag fusion Localizer for the follower if you want to use this by default for all your autos


        if (following && !follower.isBusy()) following = false;
    }

    private Pose getRobotPoseFromCamera() {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this

        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}*/
