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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Teleop;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;



import java.util.List;
import java.util.ArrayList;

@Autonomous(name = "Example Auto", group = "Examples")
public class Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotorEx intake;
    private PathChain pathChain;
    private AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;


    //----------------FKYWHEEL SETUP--------------------
    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;
    private double intakePower = -0.4;

    private final Pose startPose = new Pose(21.42627345844504, 121.60857908847186, Math.toRadians(135)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(53.27613941018767, 90.14477211796246, Math.toRadians(135));
    private final Pose highStart = new Pose(42.85254691689008, 83.96782841823057, Math.toRadians(180));
    private final Pose highEnd = new Pose(16.986595174262735, 83.96782841823057, Math.toRadians(180));
    private final Pose middleStart = new Pose(42.85254691689008, 60.03217158176943, Math.toRadians(180));
    private final Pose middleEnd = new Pose(16.986595174262735, 60.03217158176943, Math.toRadians(180));
    private final Pose lowStart = new Pose(42.85254691689008, 35.71045576407508, Math.toRadians(180));
    private final Pose lowEnd = new Pose(16.986595174262735, 35.71045576407508, Math.toRadians(180));
    private final Pose leavePose = new Pose(40,70,Math.toRadians(180));

    private static final double FAST_VEL = 30;   // in/s (travel)

    private static final double SLOW_VEL = 25;   // in/s (intaking / precision)

    // -------------- APRILTAG --------------
    private boolean tagCorrectionEnabled = true;

    private boolean tagIsValid = false;
    private Pose tagPose = null;

    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final Pose BLUE_GOAL_TAG_POSE = new Pose(72,144,Math.toRadians(180));

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
    private Path getToHighStart, grabHigh, scoreHigh, getToMiddleStart, grabMiddle, scoreMiddle, getToLowStart, grabLow, scoreLow, leavePoint;


    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */


        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setVelocityConstraint(FAST_VEL);


        getToHighStart = new Path(new BezierLine(scorePose, highStart));
        getToHighStart.setLinearHeadingInterpolation(scorePose.getHeading(), highStart.getHeading());
        getToHighStart.setVelocityConstraint(FAST_VEL);


//        getToHighStart = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, highStart))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), highStart.getHeading())
//                .setVelocityConstraint(FAST_VEL)
//                .build();



        grabHigh = new Path(new BezierLine(highStart, highEnd));
        grabHigh.setLinearHeadingInterpolation(highStart.getHeading(), highEnd.getHeading());
        grabHigh.setVelocityConstraint(SLOW_VEL);


//        grabHigh = follower.pathBuilder()
//                .addPath(new BezierLine(highStart, highEnd))
//                .setLinearHeadingInterpolation(highStart.getHeading(), highEnd.getHeading())
//                .setVelocityConstraint(SLOW_VEL)
//                .build();

        scoreHigh = new Path(new BezierLine(highEnd, scorePose));
        scoreHigh.setLinearHeadingInterpolation(highEnd.getHeading(), scorePose.getHeading());
        scoreHigh.setVelocityConstraint(FAST_VEL);


//        scoreHigh = follower.pathBuilder()
//                .addPath(new BezierLine(highEnd, scorePose))
//                .setLinearHeadingInterpolation(highEnd.getHeading(), scorePose.getHeading())
//                .setVelocityConstraint(FAST_VEL)
//                .build();


        getToMiddleStart = new Path(new BezierLine(scorePose, middleStart));
        getToMiddleStart.setLinearHeadingInterpolation(scorePose.getHeading(), middleStart.getHeading());
        getToMiddleStart.setVelocityConstraint(FAST_VEL);

//        getToMiddleStart = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, middleStart))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), middleStart.getHeading())
//                .setVelocityConstraint(FAST_VEL)
//                .build();

        grabMiddle = new Path(new BezierLine(middleStart, middleEnd));
        grabMiddle.setLinearHeadingInterpolation(middleStart.getHeading(), middleEnd.getHeading());
        grabMiddle.setVelocityConstraint(SLOW_VEL);


//        grabMiddle = follower.pathBuilder()
//                .addPath(new BezierLine(middleStart, middleEnd))
//                .setLinearHeadingInterpolation(middleStart.getHeading(), middleEnd.getHeading())
//                .setVelocityConstraint(SLOW_VEL)
//                .build();

        scoreMiddle = new Path(new BezierLine(middleEnd, scorePose));
        scoreMiddle.setLinearHeadingInterpolation(middleEnd.getHeading(), scorePose.getHeading());
        scoreMiddle.setVelocityConstraint(FAST_VEL);


//        scoreMiddle = follower.pathBuilder()
//                .addPath(new BezierLine(middleEnd, scorePose))
//                .setLinearHeadingInterpolation(middleEnd.getHeading(), scorePose.getHeading())
//                .setVelocityConstraint(FAST_VEL)
//                .build();

        getToLowStart = new Path(new BezierLine(scorePose, lowStart));
        getToLowStart.setLinearHeadingInterpolation(scorePose.getHeading(), lowStart.getHeading());
        getToLowStart.setVelocityConstraint(FAST_VEL);


//        getToLowStart = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, lowStart))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), lowStart.getHeading())
//                .setVelocityConstraint(FAST_VEL)
//                .build();

        grabLow = new Path(new BezierLine(lowStart, lowEnd));
        grabLow.setLinearHeadingInterpolation(lowStart.getHeading(), lowEnd.getHeading());
        grabLow.setVelocityConstraint(SLOW_VEL);


//        grabLow = follower.pathBuilder()
//                .addPath(new BezierLine(lowStart, lowEnd))
//                .setLinearHeadingInterpolation(lowStart.getHeading(), lowEnd.getHeading())
//                .setVelocityConstraint(SLOW_VEL)
//                .build();

        scoreLow = new Path(new BezierLine(lowEnd, scorePose));
        scoreLow.setLinearHeadingInterpolation(lowEnd.getHeading(), scorePose.getHeading());
        scoreLow.setVelocityConstraint(FAST_VEL);

//        scoreLow = follower.pathBuilder()
//                .addPath(new BezierLine(lowEnd, scorePose))
//                .setLinearHeadingInterpolation(lowEnd.getHeading(), scorePose.getHeading())
//                .setVelocityConstraint(FAST_VEL)
//                .build();

        leavePoint = new Path(new BezierLine(scorePose, leavePose));
        leavePoint.setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading());
        leavePoint.setVelocityConstraint(FAST_VEL);


//        leavePoint = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, leavePose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
//                .setVelocityConstraint(FAST_VEL)
//                .build();

    }




    private void updateAprilTagPose() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        tagIsValid = false;
        tagPose = null;

        AprilTagDetection best = null;

        for (AprilTagDetection d : detections) {
            if (d.id == BLUE_GOAL_TAG_ID) {
                if (best == null || d.ftcPose.range < best.ftcPose.range) {
                    best = d;
                }
            }
        }

        if (best == null) return;

        double range = best.ftcPose.range;
        double bearing = Math.toRadians(best.ftcPose.bearing);
        double yaw = Math.toRadians(best.ftcPose.yaw);

        if (range > 72 || Math.abs(best.ftcPose.yaw) > 20) return;

        double relX = range * Math.cos(bearing);
        double relY = range * Math.sin(bearing);

        double tagH = BLUE_GOAL_TAG_POSE.getHeading();

        double fieldX = BLUE_GOAL_TAG_POSE.getX()
                - (relX * Math.cos(tagH) - relY * Math.sin(tagH));
        double fieldY = BLUE_GOAL_TAG_POSE.getY()
                - (relX * Math.sin(tagH) + relY * Math.cos(tagH));

        double fieldH = tagH + Math.PI - yaw;

        tagPose = new Pose(fieldX, fieldY, fieldH);
        tagIsValid = true;
    }


    private void correctPoseWithAprilTag() {
        if (!tagIsValid || tagPose == null) return;

        Pose current = follower.getPose();

        double x = 0.7 * current.getX() + 0.3 * tagPose.getX();
        double y = 0.7 * current.getY() + 0.3 * tagPose.getY();

        double deltaH = tagPose.getHeading() - current.getHeading();
        while (deltaH > Math.PI) deltaH -= 2 * Math.PI;
        while (deltaH < -Math.PI) deltaH += 2 * Math.PI;

        double h = current.getHeading() + 0.3 * deltaH;

        follower.setPose(new Pose(x, y, h));
    }


        public void autonomousPathUpdate() {
        switch (pathState) {

            case SCORE_PRELOAD:
                follower.followPath(scorePreload);
                pathState = PathState.SHOOT_PRELOAD;
                break;


            case SHOOT_PRELOAD:


                if (!follower.isBusy()) {
                   /* if (tagIsValid) {
                        correctPoseWithAprilTag(tagPose);
                    } */
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        follower.followPath(getToHighStart);
                        shotsTriggered = false;
                        intake.setPower(intakePower);
                        pathState = PathState.GET_TO_HIGH_START;
                    }
                }

                break;


            case GET_TO_HIGH_START:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    follower.followPath(grabHigh);
                    pathState = PathState.GRAB_HIGH;
                }
                break;
            case GRAB_HIGH:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Sample */
                    intake.setPower(0);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scoreHigh);
                    pathState = PathState.SCORE_HIGH;
                }
                break;
            case SCORE_HIGH:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* if (tagIsValid && !follower.isBusy()) {
                        correctPoseWithAprilTag(tagPose);
                    } */
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        follower.followPath(getToMiddleStart);
                        shotsTriggered = false;
                        intake.setPower(intakePower);
                        pathState = PathState.GET_TO_MIDDLE_START;
                    }
                }

                break;
            case GET_TO_MIDDLE_START:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */


                if (!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabMiddle);
                    pathState = PathState.GRAB_MIDDLE;
                }
                break;
            case GRAB_MIDDLE:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    intake.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scoreMiddle);
                    pathState = PathState.SCORE_MIDDLE;
                }
                break;
            case SCORE_MIDDLE:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* if (tagIsValid && !follower.isBusy()) {
                        correctPoseWithAprilTag(tagPose);
                    } */
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        follower.followPath(getToLowStart);
                        intake.setPower(intakePower);
                        shotsTriggered = false;
                        pathState = PathState.GET_TO_LOW_START;
                    }
                }
                break;
            case GET_TO_LOW_START:

                if (!follower.isBusy()) {

                    follower.followPath(grabLow);
                    pathState = PathState.GRAB_LOW;
                }
                break;
            case GRAB_LOW:

                if (!follower.isBusy()) {

                    intake.setPower(0);

                    follower.followPath(scoreLow);
                    pathState = PathState.SCORE_LOW;
                }
                break;
            case SCORE_LOW:

                if (!follower.isBusy()) {

                    /*if (tagIsValid && !follower.isBusy()) {
                        correctPoseWithAprilTag(tagPose);
                    }*/

                    follower.followPath(leavePoint);
                    pathState = PathState.LEAVE_POINT;
                }
                break;
            case LEAVE_POINT:

                if (!follower.isBusy()) {
                    Teleop.startingPose = follower.getPose();
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
        List<AprilTagDetection> detections = aprilTag.getFreshDetections();


        tagIsValid = false;      // reset
        tagPose = null;

        AprilTagDetection bestDetection = null;
        /* April

        for (AprilTagDetection detection : detections) {
            if (detection.id == BLUE_GOAL_TAG_ID) {
                if (bestDetection == null ||
                        detection.ftcPose.range < bestDetection.ftcPose.range) {
                    bestDetection = detection;
                }
            }
        }

        if (bestDetection != null) {

            double range = bestDetection.ftcPose.range;        // inches
            double bearing = Math.toRadians(bestDetection.ftcPose.bearing);
            double yaw = Math.toRadians(bestDetection.ftcPose.yaw);

            // Camera offset on robot (example values)
            double CAMERA_FORWARD = 6.0; // inches
            double CAMERA_LEFT = 0.0;
            double CAMERA_HEADING_OFFSET = 0.0;

            // Robot position relative to tag
            double relX = range * Math.cos(bearing);
            double relY = range * Math.sin(bearing);

            double tagHeading = BLUE_GOAL_TAG_POSE.getHeading();

            double fieldX = BLUE_GOAL_TAG_POSE.getX()
                    - (relX * Math.cos(tagHeading) - relY * Math.sin(tagHeading))
                    - CAMERA_FORWARD;

            double fieldY = BLUE_GOAL_TAG_POSE.getY()
                    - (relX * Math.sin(tagHeading) + relY * Math.cos(tagHeading))
                    - CAMERA_LEFT;

            double fieldHeading = tagHeading + Math.PI - yaw + CAMERA_HEADING_OFFSET;

            tagPose = new Pose(fieldX, fieldY, fieldHeading);

            tagIsValid = (range < 72) && (Math.abs(Math.toDegrees(yaw)) < 20);
        } */
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
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();


        shooter.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        pathState = PathState.SCORE_PRELOAD;



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
