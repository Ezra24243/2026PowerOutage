package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class FlywheelLogic {
    public DcMotorEx Lizard;
    public Servo flipper;

    private ElapsedTime stateTimer = new ElapsedTime();

    public enum FlywheelState {
        IDLE,
        WARMING_UP,
        FLIPPING,
        CHECK
    }

    private FlywheelState flywheelState;

    //----------------  FLYWHEEL CONSTANTS  ------------------

    public int shotsRemaining = 0;
    private static final double MIN_TPS = 6500;
    private static final double TARGET_TPS = 7000;
    private static final double MAX_WARMUP_TIME = 1.2;



    //----------------  FLIPPER CONSTANTS ----------------

    private static final double FLIPPER_DOWN = 0.6;
    private static final double FLIPPER_FLIPPED = 0;
    private static final double DOWN_TIME = 0.75; //time for next ball to roll into position
    private static final double UP_TIME = 0.25; //time for flipper to keep supporting lifted ball


    public void init(HardwareMap hwMap) {
        Lizard = hwMap.get(DcMotorEx.class, "Lizard");
        flipper = hwMap.get(Servo.class, "flipper");

        //tune PIDF
        Lizard.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lizard.setVelocity(0);

        flywheelState = FlywheelState.IDLE;
        flipper.setPosition(FLIPPER_DOWN);
    }

    public void update() {

        double rpm = Lizard.getVelocity();

        switch (flywheelState) {

            case IDLE:
                Lizard.setVelocity(0);
                flipper.setPosition(FLIPPER_DOWN);
//                if (shotsRemaining > 0) {
//                    flipper.setPosition(FLIPPER_DOWN);
//                    Lizard.setVelocity(TARGET_RPM);
//                    stateTimer.reset();
//                    flywheelState = FlywheelState.WARMING_UP;
//                }
//                else {
//                    Lizard.setVelocity(0);
//                }
                break;
            case WARMING_UP:
                Lizard.setVelocity(TARGET_TPS);
                if (rpm > MIN_TPS || stateTimer.seconds() > MAX_WARMUP_TIME) {
                    stateTimer.reset();
                    flywheelState = FlywheelState.FLIPPING;
                }
                break;
            case FLIPPING:

                flipper.setPosition(FLIPPER_FLIPPED);

                if (stateTimer.seconds() > UP_TIME) {
                    flipper.setPosition(FLIPPER_DOWN);
                    stateTimer.reset();
                    shotsRemaining--;

                    flywheelState = FlywheelState.CHECK;
                }
                break;


            case CHECK:
                if (stateTimer.seconds() > DOWN_TIME) {
                    if (shotsRemaining > 0) {
                        stateTimer.reset();
                        flipper.setPosition(FLIPPER_DOWN);
                        flywheelState = FlywheelState.WARMING_UP;
                    }

                    else {
                        flywheelState = FlywheelState.IDLE;
                    }
                }
                break;

        }
    }
    public void fireShots(int numberOfShots) {
        if (flywheelState == FlywheelState.IDLE && numberOfShots > 0) {
            shotsRemaining = numberOfShots;
            stateTimer.reset();
            flywheelState = FlywheelState.WARMING_UP;
        }
    }

   public void cancel() {
        shotsRemaining = 0;
        flywheelState = FlywheelState.IDLE;
   }

    public boolean isBusy() {

        return flywheelState != FlywheelState.IDLE;
    }

    public String getState() {
        return flywheelState.name();
    }
}
