package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Drivetrain {
    final double FORWARD_TICKS_PER_CM = 13;
    final double SIDEWAYS_TICKS_PER_CM = 13;
    //82.5
    double currentY = 0;
    double lastY = 0;

    double currentX = 0;
    double lastX = 0;
    double remainingY = 0;
    double remaningX = 0;

    double currentPosTicks = 0;
    double remainingPosTicks = 0;
    double lastPosTicks = 0;

    double currentNegTicks = 0;
    double remainingNegTicks = 0;
    double lastNegTicks = 0;
    private LiftSystem lift;
    private DcMotor liftMotor;


    //Initialize the Motors
    //L = Left
    //R = Right
    //B = Back
    //F = Front
    private DcMotor FLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor BRMotor = null;
    private DcMotor BootMotor = null;


    public Drivetrain(HardwareMap hardwareMap) {
        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        FRMotor = hardwareMap.get(DcMotor.class, "FR");
        BLMotor = hardwareMap.get(DcMotor.class, "BL");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");

    }

    public void init() {

        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // No dead wheel

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public double deadWheelFunctionY(double totalY) {

        //Y data
        totalY *= FORWARD_TICKS_PER_CM;
        resetEncoders();
        //noinspection IntegerDivisionInFloatingPointContext
        currentY = (BRMotor.getCurrentPosition() + FRMotor.getCurrentPosition()) / 2;
        remainingY = Math.abs(totalY) - Math.abs(currentY);
        lastY = currentY;
        remainingY /= FORWARD_TICKS_PER_CM;
        return remainingY;
    }

    /**
     * @param totalX the total x that needs to be traveled
     **/
    public double deadWheelFuncitionX(double totalX) {
        //X Data
        totalX *= SIDEWAYS_TICKS_PER_CM;
        resetEncoders();
        currentX = BLMotor.getCurrentPosition();
        remaningX = Math.abs(totalX) - Math.abs(currentX);
        lastX = currentX;
        remaningX /= SIDEWAYS_TICKS_PER_CM;
        return remaningX;
    }

    /**
     * @param posTicks the initatal amount of ticks to go
     * @return remaing ticks to go
     */
    public double deadWheelFunctionLeft(double posTicks) {
        resetEncoders();
        currentPosTicks = BRMotor.getCurrentPosition();
        remainingPosTicks = Math.abs(posTicks) - Math.abs(currentPosTicks);
        lastPosTicks = currentPosTicks;
        return remainingPosTicks;
    }

    /**
     * @param negTicks the initatal amount of ticks to go
     * @return the amount of ticks remaining
     */
    public double deadWheelFunctionRight(double negTicks) {
        resetEncoders();
        currentNegTicks = FRMotor.getCurrentPosition();
        remainingNegTicks = Math.abs(negTicks) - Math.abs(currentNegTicks);
        lastNegTicks = currentNegTicks;
        return remainingNegTicks;
    }

    /**
     * @param direction the direciton of the robot 1 is backwards -1 is forward
     * @param speed     the speed of the robot
     * @param distanceY the remaining distance of Y
     * @param tf        the tolerance factor
     */
    public void linearDrive(double direction, double speed, double distanceY, double tf) {
        while (distanceY >= tf) {
            FLMotor.setPower(speed * direction);
            FRMotor.setPower(speed * direction);
            BLMotor.setPower(speed * direction);
            BRMotor.setPower(speed * direction);

            double tempDistanceY = distanceY;
            distanceY = deadWheelFunctionY(tempDistanceY);

            telemetry.addData("remainingY", remainingY);
            telemetry.addData("FL", FLMotor.getPower());
            telemetry.addData("FR", FRMotor.getPower());
            telemetry.addData("BL", BLMotor.getPower());
            telemetry.addData("BR", BRMotor.getPower());
            telemetry.addData("tempy", tempDistanceY);
            telemetry.addData("ydsit", distanceY);
            telemetry.update();
        }

        FLMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        BRMotor.setPower(0);

    }


    /**
     * @param direction the direciton of the robot, 1 is left, -1 is right
     * @param speed     the speed of the robot
     * @param distanceX the remaining distance of X
     * @param tf        the tolerance factor
     */
    public void strafingDrive(double direction, double speed, double distanceX, double tf) {
        while (distanceX >= tf) {
            FLMotor.setPower(speed * direction);
            FRMotor.setPower(speed * -direction);
            BLMotor.setPower(speed * -direction);
            BRMotor.setPower(speed * direction);

            double tempDistanceX = distanceX;
            distanceX = deadWheelFuncitionX(tempDistanceX);


            telemetry.addData("remainingX", distanceX);
            telemetry.addData("FL", FLMotor.getPower());
            telemetry.addData("FR", FRMotor.getPower());
            telemetry.addData("BL", BLMotor.getPower());
            telemetry.addData("BR", BRMotor.getPower());
            telemetry.addData("tempX", tempDistanceX);
            telemetry.addData("Xdsit", distanceX);
            telemetry.update();
        }

        FLMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        BRMotor.setPower(0);

    }

    /**
     * @param direction   the direciton of the robot, -1 is right, 1 is left
     * @param speed       the speed of the robot
     * @param targetAngle the remaining distance of X
     * @param tf          the tolerance factor
     */
    public void rotationDrive(double direction, double speed, double targetAngle, double tf) {
        // Convert targetAngle to encoder ticks based on your robot's geometry
        double ticksPerDegree = 15; // Calculate or determine this based on wheel geometry and motor specs
        double targetTicks = targetAngle * ticksPerDegree;

        // Reset encoders before starting the rotation
        resetEncoders();

        while (targetTicks >= tf) {
            // Set motor powers for rotation
            FLMotor.setPower(direction * speed);
            BLMotor.setPower(direction * speed);
            FRMotor.setPower(-direction * speed);
            BRMotor.setPower(-direction * speed);

            // Get the difference in encoder ticks between left and right sides
            double leftTicks = FLMotor.getCurrentPosition();
            double rightTicks = FRMotor.getCurrentPosition();

            // Track rotation by using the difference between left and right encoder values
            double currentRotationTicks = Math.abs(leftTicks - rightTicks) / 2;

            // Update the remaining target ticks
            targetTicks -= currentRotationTicks;

            // Telemetry for debugging
            telemetry.addData("Remaining Ticks", targetTicks);
            telemetry.addData("Left Ticks", leftTicks);
            telemetry.addData("Right Ticks", rightTicks);
            telemetry.update();

            // Reset encoders after each loop iteration (optional depending on robot behavior)
            resetEncoders();
        }

        // Stop motors after rotation
        FLMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        BRMotor.setPower(0);


    }

    private void resetEncoders() {

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void moveLift(int pos) {
        liftMotor.setTargetPosition(pos);
        liftMotor.setPower(1);

        while (liftMotor.isBusy()) {
            telemetry.addData("abc", liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    public void teleopMove() {
        //This drive code uses a drive system where the left joystick controls Forward/Backward Left/Right
        //And the right joystick rotates the robot on it's axis

        double speedFactor = 1;
        double zAxisInput = -gamepad1.right_stick_x; //Rotation on the axis input
        double xAxisInput = -gamepad1.left_stick_x; //Strafing axis
        double yAxisInput = gamepad1.left_stick_y; //Forward/Reverse axis

        //Combine joystick inputs for the power to the wheels
        //Google a diagram of how the math works
        //Too long to write here, but not crazy complicated :D

        double FLPower = (yAxisInput + xAxisInput + zAxisInput) * speedFactor;
        double FRPower = (yAxisInput - xAxisInput - zAxisInput) * speedFactor;
        double BLPower = (yAxisInput - xAxisInput + zAxisInput) * speedFactor;
        double BRPower = (yAxisInput + xAxisInput - zAxisInput) * speedFactor;

        //Ensure that the power value does not exceed 1 for the motors
        //Don't want to blow up a motor; good idea luke -EG

        //Declare a max variable for the math. Will be set to 1 during the program
        double max;
        max = Math.max(Math.abs(FLPower), Math.abs(FRPower));
        max = Math.max(max, Math.abs(BLPower));
        max = Math.max(max, Math.abs(BRPower));

        //Divide all the powers by the max so that the max power is 1
        if (max > 1.0) {
            FLPower /= max;
            FRPower /= max;
            BLPower /= max;
            BRPower /= max;
        }

        //Send the power to the motors with the variables defined above
        FLMotor.setPower(FLPower);
        FRMotor.setPower(FRPower);
        BLMotor.setPower(BLPower);
        BRMotor.setPower(BRPower);

        if (gamepad1.right_bumper) {
            speedFactor = .5;
        }
        if (gamepad1.left_bumper) {
            speedFactor = 1;
        }

        telemetry.addData("FR Power", FRPower);
        telemetry.addData("FL Power", FLPower);
        telemetry.addData("BR Power", BRPower);
        telemetry.addData("BL Power", BLPower);

        resetEncoders();
        BootMotor.setTargetPosition(15);
        BootMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BootMotor.setPower(1);
    }
}