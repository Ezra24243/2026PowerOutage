package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name = "TimeAuto", group = "Examples")
public class TimeAuto extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotorEx intake;
    private PathChain pathChain;
    private AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotorEx bLeft;
    private DcMotorEx bRight;



    //----------------FKYWHEEL SETUP--------------------
    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;
    private double intakePower = -0.4;

    private static final double FAST_VEL = 30;   // in/s (travel)

    private static final double SLOW_VEL = 25;   // in/s (intaking / precision)

    @Override
    public void runOpMode() {

















        /**
         * This method is called once at the init of the OpMode.
         **/


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        bLeft = hardwareMap.get(DcMotorEx.class, "bLeft");
        bRight = hardwareMap.get(DcMotorEx.class, "bRight");


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







        /**
         * This method is called continuously after Init while waiting for "play".
         **/


        /**
         * This method is called once at the start of the OpMode.
         * It runs all the setup actions, including building paths and starting the path system
         **/
        opmodeTimer.resetTimer();

        waitForStart();


        boolean back = true;
        bLeft.setPower(-0.5);
        bRight.setPower(-0.5);
        sleep(1000);
        bLeft.setPower(0);
        bRight.setPower(0);
        back = false;
        boolean turn = false;
        shooter.fireShots(3);
        if (!shooter.isBusy() && !back) {
            turn = true;
            bLeft.setPower(-0.5);
            bRight.setPower(0.5);
            sleep(1000);
            bLeft.setPower(0);
            bRight.setPower(0);
            turn = false;
        }

        if (!turn && !back && !shooter.isBusy()) {
            bLeft.setPower(0.5);
            bRight.setPower(0.5);
            sleep(500);
            bLeft.setPower(0);
            bRight.setPower(0);
        }





    }

}

/** We do not use this because everything should automatically disable **/
