package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;




public class LiftSystem {
    // methods to go up and down
    private final double LIFT_ENCODER_RES = ((((1+(46d/17))) * (1+(46d/17))) * 28);

    public int liftPos = 0;
    public enum LiftPosition {
        RESTING(0),
        RUNG_LOW(1447),
        RUNG_HIGH(1895),
        BASKET_LOW(0),
        BASKET_HIGH(0);

        LiftPosition(int encoderPos) {
            this.encoderPos = encoderPos;
        }

        public final int encoderPos;


    }
    DcMotor liftMotor;

    Servo outtake;

    public LiftSystem(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.dcMotor.get("lift");
        outtake = hardwareMap.servo.get("outtake");
    }

    public void stopAndResetEncoders() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void init() {
        stopAndResetEncoders();

        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake.scaleRange(0, 1);
    }



    public void setLiftTo(LiftPosition liftPosition) {

        int currentPosition = liftMotor.getCurrentPosition();
        int targetPosition = liftPosition.encoderPos;
        liftPos = liftMotor.getCurrentPosition();

        // Determine direction based on position difference
        double motorPower = targetPosition > currentPosition ? 1 : -1;

        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(motorPower);
    }

    public void setLiftTo(int liftEncPosition) {
        liftMotor.setPower(1);
        liftMotor.setTargetPosition(liftEncPosition);
    }

    public int getLiftTargetPosition() {
        return liftMotor.getTargetPosition();
    }

    public void waitForMovement() {
        while (liftMotor.isBusy()) {
            Thread.yield();
        }

        liftMotor.setPower(0);
    }

    public void waitForMovement(long timeoutMillis) {
        long startTime = System.currentTimeMillis();

        while (liftMotor.isBusy() && (System.currentTimeMillis() - startTime) < timeoutMillis) {
            Thread.yield();

        }

        liftMotor.setPower(0);
    }

    public void closeOuttake() {
        outtake.setPosition(0d);
    }

    public void openOuttake() {
        outtake.setPosition(0.75);
    }



    private void moveLift(int pos) {
        liftMotor.setTargetPosition(pos);
        liftMotor.setPower(1);


    }
}
