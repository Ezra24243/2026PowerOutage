package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.Auto;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.function.Supplier;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.FlywheelLogic;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;
import com.qualcomm.robotcore.util.ElapsedTime;


@Configurable
@TeleOp
public class Teleop extends OpMode {

    private double flipperDown = 0.5;
    private double flipperUp = 0.1;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> shoot1;
    private Supplier<PathChain> shoot2;
    private Supplier<PathChain> park;

    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;
    private boolean intaking;


    private TelemetryManager telemetryM;
    private DcMotor bLeft;
    private DcMotor bRight;
    private DcMotor fLeft;
    private DcMotor fRight;
    private DcMotor intake;
    private Servo flipper;
    private boolean lastDown = false;
    private boolean lastUp = false;
    private boolean lastLB = false; //This is for gamepad1
    private boolean lastDown1 = false;
    private boolean lastUp1 = false;
    private boolean lastLeft = false;

    @Override
    public void init() {



        shooter.init(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flipper = hardwareMap.get(Servo.class, "flipper");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        shoot1 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(150), 1))

                .build();

        shoot2 = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(150), 1))
                .build();

        park = () -> follower.pathBuilder()
                .addPath( new Path(new BezierLine(follower::getPose, new Pose(38.5, 34))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 1))
                .build();

        intaking = false;




        telemetry.addLine("Peanut or Done");
        telemetry.update();


    }


    @Override
    public void start() {


        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");


        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();

//        automatedDrive = true;
    }

    @Override
    public void loop() {

        //Call this once per loop
        follower.update();
//        telemetryM.update();
        shooter.update();


//DRIVER 1'S CONTROLLER
        if (!automatedDrive) follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
             -gamepad1.right_stick_x,
            true // Robot Centric
            );

//        if (!automatedDrive) {
//            bRight.setPower(-gamepad1.right_stick_y);
//            fLeft.setPower(-gamepad1.left_stick_y);
//        }
        //Automated PathFollowing
        if (!automatedDrive && gamepad1.dpad_left && !lastLeft) {
            follower.followPath(shoot1.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done


        if (!automatedDrive && gamepad1.dpad_up && !lastUp1) {
            follower.followPath(shoot2.get());
            automatedDrive = true;
        }

        if (!automatedDrive && gamepad1.dpad_down && !lastDown1) {
            follower.followPath(park.get());

            automatedDrive = true;
        }

        lastLeft = gamepad1.dpad_left;
        lastUp1 = gamepad1.dpad_up;
        lastDown1 = gamepad1.dpad_down;

        if (automatedDrive && (gamepad1.right_bumper || !follower.isBusy())) {
            follower.startTeleopDrive();
            flipper.setPosition(flipperDown);
            automatedDrive = false;
        }





        telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);

//DRIVER 2'S CONTROLLER

        if (gamepad2.dpad_down && !lastDown) {
            shotsTriggered = true;
            shooter.fireShots(3);
        }

        if (gamepad2.dpad_up && !lastUp) {
            shotsTriggered = false;
            shooter.cancel();
        }

        lastDown = gamepad2.dpad_down;
        lastUp = gamepad2.dpad_up;

       if (gamepad1.left_bumper && !lastLB) {
           intaking = !intaking;
       }
        lastLB = gamepad1.left_bumper;


        if (intaking) {
           intake.setPower(-0.6);
       }
       else {
           intake.setPower(0);
       }

       telemetry.addData("Shooter State", shooter.getState());
       telemetry.addData("Shooter Busy", shooter.isBusy());
       telemetry.addData("Intaking", intaking);
       telemetry.addData("Flywheel Velocity", shooter.Lizard.getVelocity());
       telemetry.addData("Shots Remaining", shooter.shotsRemaining);
       telemetry.update();

    }


}
