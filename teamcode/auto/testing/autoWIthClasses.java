package org.firstinspires.ftc.teamcode.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.LiftSystem;


@Autonomous(name="autoClasses", group="Testing")


public class autoWIthClasses extends LinearOpMode {
    private LiftSystem lift;
    private Drivetrain drivetrain;

    public void runOpMode() {
        lift = new LiftSystem(hardwareMap);
        lift.init();

        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.init();

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.linearDrive(-1,1,230,6);
            sleep(100);
            //drive to drop spot

            lift.setLiftTo(1800);
            //raise lift

            drivetrain.linearDrive(-1,.5,45,3);
            sleep(100);
            //get closer

            lift.setLiftTo(1200);
            sleep(100);
            lift.openOuttake();
            sleep(600);
            //release

            drivetrain.linearDrive(1,.5,45,3);
            //back away from rig

            lift.setLiftTo(10);
            //lower lift

            drivetrain.rotationDrive(-1,1,85,.3);
            sleep(100);
            //rotate towards pickup

            drivetrain.linearDrive(-1,1,375,.3);
            sleep(100);
            //drive towards pickup

            drivetrain.rotationDrive(-1,1,75,.3);
            sleep(100);
            //rotate towards pickup

            drivetrain.linearDrive(-1,.75,200,.3);
            sleep(100);
            //drive closer to pickup

            lift.closeOuttake();
            sleep(150);
            lift.closeOuttake();
            sleep(400);
            //grab spec.

            drivetrain.linearDrive(1,1,150,.3);
            sleep(100);
            //back away from spec

            drivetrain.rotationDrive(-1,1,90,.3);
            sleep(100);
            //rotate towards rig

            drivetrain.linearDrive(-1,1,500,.3);
            sleep(100);
            //drive to rig

            drivetrain.rotationDrive(-1,1,80,.3);
            sleep(100);
            //rotate perpendicular to the rig

            lift.setLiftTo(1800);
            //raise lift

            drivetrain.linearDrive(-1,1,127,.3);
            sleep(100);
            //drive to rig

            lift.setLiftTo(1300);
            sleep(100);
            //lower lift

            lift.openOuttake();
            sleep(100);
            //open lift

            drivetrain.linearDrive(1,1,80,.3);
            sleep(100);
            //back away from rig

            lift.setLiftTo(30);
            //lower lift

            drivetrain.rotationDrive(-1,1,80,.3);
            sleep(100);
            //rotate towards park

            drivetrain.linearDrive(-1,1,500,.3);
            sleep(100);
            //get close to park place

            drivetrain.strafingDrive(-1,1,29,.3);
            //straff to parking
            
        }
    }

}
