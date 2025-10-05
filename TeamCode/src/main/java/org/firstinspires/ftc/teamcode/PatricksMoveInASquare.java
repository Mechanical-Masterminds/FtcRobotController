package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Move In A Square Auto Test")
public class PatricksMoveInASquare extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startpose = new Pose(72,72,Math.toRadians(90));
    private final Pose up = new Pose(72,96,Math.toRadians(90));
    private final Pose right = new Pose(96,96, Math.toRadians(90));
    private final Pose Down = new Pose(96,72,Math.toRadians(90));
    private final Pose left = new Pose(72,72,Math.toRadians(90));
    private PathChain squarePath;
    public void buildPaths() {

        squarePath = follower.pathBuilder()
                .addPath(new BezierLine(startpose, up))
                .setLinearHeadingInterpolation(startpose.getHeading(), up.getHeading())
                .addPath(new BezierLine(up, right))
                .setLinearHeadingInterpolation(up.getHeading(), right.getHeading())
                .addPath(new BezierLine(right, Down))
                .setLinearHeadingInterpolation(right.getHeading(), Down.getHeading())
                .addPath(new BezierLine(Down, left))
                .setLinearHeadingInterpolation(Down.getHeading(), left.getHeading())
                .build();


    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(squarePath);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startpose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}