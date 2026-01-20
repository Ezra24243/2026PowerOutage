package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Example Auto", group = "Examples")
public class Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotorEx intake;


    //----------------FKYWHEEL SETUP--------------------
    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;

    private final Pose startPose = new Pose(21.42627345844504, 121.60857908847186, Math.toRadians(135)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(53.27613941018767, 90.14477211796246, Math.toRadians(135));
    private final Pose highStart = new Pose(42.85254691689008, 83.96782841823057, Math.toRadians(180));
    private final Pose highEnd = new Pose(16.986595174262735, 83.96782841823057, Math.toRadians(180));
    private final Pose middleStart = new Pose(42.85254691689008, 60.03217158176943, Math.toRadians(180));
    private final Pose middleEnd = new Pose(16.986595174262735, 60.03217158176943, Math.toRadians(180));
    private final Pose lowStart = new Pose(42.85254691689008, 35.71045576407508, Math.toRadians(180));
    private final Pose lowEnd = new Pose(16.986595174262735, 35.71045576407508, Math.toRadians(180));
    private final Pose leavePose = new Pose(30,70,Math.toRadians(180));

    private enum PathState {
        SCORE_PRELOAD,
        SHOOT_PRELOAD,
        GET_TO_HIGH_START,
        GRAB_HIGH,
        SCORE_HIGH,
        SHOOT_HIGH,
        GET_TO_MIDDLE_START,
        GRAB_MIDDLE,
        SCORE_MIDDLE,
        SHOOT_MIDDLE,
        GET_TO_LOW_START,
        GRAB_LOW,
        SCORE_LOW,
        SHOOT_LOW,
        LEAVE_POINT
    }

    private PathState pathState;

    private Path scorePreload;
    private PathChain getToHighStart, grabHigh, scoreHigh, getToMiddleStart, grabMiddle, scoreMiddle, getToLowStart, grabLow, scoreLow, leavePoint;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());


        getToHighStart = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, highStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), highStart.getHeading())
                .build();

        grabHigh = follower.pathBuilder()
                .addPath(new BezierLine(highStart, highEnd))
                .setLinearHeadingInterpolation(highStart.getHeading(), highEnd.getHeading())
                .build();

        scoreHigh = follower.pathBuilder()
                .addPath(new BezierLine(highEnd, scorePose))
                .setLinearHeadingInterpolation(highEnd.getHeading(), scorePose.getHeading())
                .build();

        getToMiddleStart = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, middleStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), middleStart.getHeading())
                .build();

        grabMiddle = follower.pathBuilder()
                .addPath(new BezierLine(middleStart, middleEnd))
                .setLinearHeadingInterpolation(middleStart.getHeading(), middleEnd.getHeading())
                .build();

        scoreMiddle = follower.pathBuilder()
                .addPath(new BezierLine(middleEnd, scorePose))
                .setLinearHeadingInterpolation(middleEnd.getHeading(), scorePose.getHeading())
                .build();

        getToLowStart = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, lowStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lowStart.getHeading())
                .build();

        grabLow = follower.pathBuilder()
                .addPath(new BezierLine(lowStart, lowEnd))
                .setLinearHeadingInterpolation(lowStart.getHeading(), lowEnd.getHeading())
                .build();

        scoreLow = follower.pathBuilder()
                .addPath(new BezierLine(lowEnd, scorePose))
                .setLinearHeadingInterpolation(lowEnd.getHeading(), scorePose.getHeading())
                .build();

        leavePoint = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case SCORE_PRELOAD:
                follower.followPath(scorePreload);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:


                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        follower.followPath(getToHighStart, true);
                        shotsTriggered = false;
                        intake.setPower(0.5);
                        setPathState(PathState.GET_TO_HIGH_START);
                    }
                }

                break;
            case GET_TO_HIGH_START:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    follower.followPath(grabHigh, true);
                    setPathState(PathState.GRAB_HIGH);
                }
                break;
            case GRAB_HIGH:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    intake.setPower(0);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scoreHigh, true);
                    setPathState(PathState.SCORE_HIGH);
                }
                break;
            case SCORE_HIGH:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        follower.followPath(getToMiddleStart, true);
                        shotsTriggered = false;
                        intake.setPower(0.5);
                        setPathState(PathState.GET_TO_MIDDLE_START);
                    }
                }

                break;
            case GET_TO_MIDDLE_START:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */


                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabMiddle, true);
                    setPathState(PathState.GRAB_MIDDLE);
                }
                break;
            case GRAB_MIDDLE:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    intake.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scoreMiddle, true);
                    setPathState(PathState.SCORE_MIDDLE);
                }
                break;
            case SCORE_MIDDLE:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        follower.followPath(getToLowStart, true);
                        intake.setPower(0.5);
                        shotsTriggered = false;
                        setPathState(PathState.GET_TO_LOW_START);
                    }
                }
                break;
            case GET_TO_LOW_START:

                if (!follower.isBusy()) {

                    follower.followPath(grabLow, true);
                    setPathState(PathState.GRAB_LOW);
                }
                break;
            case GRAB_LOW:

                if (!follower.isBusy()) {

                    intake.setPower(0);

                    follower.followPath(scoreLow, true);
                    setPathState(PathState.SCORE_LOW);
                }
                break;
            case SCORE_LOW:

                if (!follower.isBusy()) {

                    follower.followPath(leavePoint, true);
                    setPathState(PathState.LEAVE_POINT);
                }
                break;
            case LEAVE_POINT:

                if (!follower.isBusy()) {
                    telemetry.addLine("Peanut or Done");
                }




        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }


    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        shooter.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooter.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);



    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(PathState.SCORE_PRELOAD);
    }

}

    /** We do not use this because everything should automatically disable **/
