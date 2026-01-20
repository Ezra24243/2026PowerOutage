package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;



public class FlywheelLogic {
    public DcMotorEx rhino;
    public Servo flipper;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum FlywheelState {
        IDLE,
        WARMING_UP,
        FLIPPING,
        CHECK
    }

    private FlywheelState flywheelState;

    //----------------  FLYWHEEL CONSTANTS  ------------------

    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double MIN_FLYWHEEL_RPM = 800;
    private double TARGET_FLYWHEEL_RPM = 1100;
    private double FLYWHEEL_MAX_WARMUP_TIME = 3;


    //----------------  FLIPPER CONSTANTS ----------------

    private double FLIPPER_DOWN = 0;
    private double FLIPPER_FLIPPED = 0.5;
    private double DOWN_TIME = 1; //time for next ball to roll into position
    private double UP_TIME = 1; //time for flipper to keep supporting lifted ball


    public void init(HardwareMap hwMap) {
        rhino = hwMap.get(DcMotorEx.class, "rhino");
        flipper = hwMap.get(Servo.class, "flipper");

        //tune PIDF
        rhino.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelState = FlywheelState.IDLE;
        flipper.setPosition(FLIPPER_DOWN);
    }

    public void update() {
        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    flipper.setPosition(FLIPPER_DOWN);
                    rhino.setVelocity(TARGET_FLYWHEEL_RPM);
                    stateTimer.reset();
                    flywheelState = FlywheelState.WARMING_UP;
                }
                else {
                    rhino.setVelocity(0);
                }
                break;
            case WARMING_UP:
                if (flywheelVelocity > MIN_FLYWHEEL_RPM || stateTimer.seconds() > FLYWHEEL_MAX_WARMUP_TIME) {

                    flywheelState = FlywheelState.FLIPPING;
                }
                break;
            case FLIPPING:
                stateTimer.reset();
                flipper.setPosition(FLIPPER_FLIPPED);

                if (stateTimer.seconds() > UP_TIME) {
                    shotsRemaining--;
                    flipper.setPosition(FLIPPER_DOWN);
                    stateTimer.reset();

                    flywheelState = FlywheelState.CHECK;
                }
                break;
            case CHECK:
                if (stateTimer.seconds() > DOWN_TIME) {
                    if (shotsRemaining > 0) {
                        stateTimer.reset();
                        flywheelState = FlywheelState.FLIPPING;
                    }

                    else {
                        flywheelState = FlywheelState.IDLE;
                    }
                }
                break;

        }
    }
    public void fireShots(int numberOfShots) {
        if (flywheelState == FlywheelState.IDLE) {
            shotsRemaining = numberOfShots;
        }
    }

    public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
    }
}
