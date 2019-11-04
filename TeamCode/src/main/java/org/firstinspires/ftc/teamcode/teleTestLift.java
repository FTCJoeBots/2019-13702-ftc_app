package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 *import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 *import com.qualcomm.robotcore.hardware.DcMotor;
 *
 *
 */

/**
 * This code is written to test the lift mechanism on the 2019 13702 robot. Specifically, it will
 * operate the lift and write lift parameters to the driver station.
 *
*/
@TeleOp(name="Tele Lift Test", group="TESTING")

public class teleTestLift extends LinearOpMode {

    double forward;
    double clockwise;
    double right;
    double k;
    double power0;
    double power1;
    double power2;
    double power3;
    double max;

    boolean prevY = false;
    boolean currY;

    boolean currdpadLeft;
    boolean prevdpadLeft = false;

    boolean currdpadRight;
    boolean prevdpadRight = false;

    boolean curra1;
    boolean preva1 = false;

    boolean currx2;
    boolean prevx2 = false;

    boolean curra2;
    boolean preva2 = false;

    HardwareJoeBot2019 robot = new HardwareJoeBot2019();
    Utility13702       U = new Utility13702();

    @Override
    public void runOpMode() throws InterruptedException {

        U.init(hardwareMap, this);
        robot.init(hardwareMap, this);

        waitForStart();

        //start of loop
        while (opModeIsActive()) {


           /* telemetry.addData("rotClampServo:", U.rotClampServo.getPosition());
            telemetry.addData("ClampServo:", U.clampServo.getPosition());
            telemetry.update(); */

            //Drive Via "Analog Sticks" (Not Toggle)
            //Set initial motion parameters to Gamepad1 Inputs
            forward = -gamepad1.left_stick_y;
            right = -gamepad1.left_trigger + gamepad1.right_trigger;
            clockwise = gamepad1.right_stick_x;

            // Add a tuning constant "K" to tune rotate axis sensitivity
            k = .6;
            clockwise = clockwise * k; //Make sure the "= Clockwise" is "= -clockwise"


            // Calculate motor power
            power0 = forward + clockwise + right;
            power1 = forward - clockwise - right;
            power2 = forward + clockwise - right;
            power3 = forward - clockwise + right;

            // Normalize Wheel speeds so that no speed exceeds 1.0
            max = Math.abs(power0);
            if (Math.abs(power1) > max) {
                max = Math.abs(power1);
            }
            if (Math.abs(power2) > max) {
                max = Math.abs(power2);
            }
            if (Math.abs(power3) > max) {
                max = Math.abs(power3);
            }

            if (max > 1) {
                power0 /= max;
                power1 /= max;
                power2 /= max;
                power3 /= max;
            }

            robot.motor0.setPower(power0);
            robot.motor1.setPower(power1);
            robot.motor2.setPower(power2);
            robot.motor3.setPower(power3);


            curra2 = gamepad2.a;
            if(curra2 && curra2 != preva2){
                U.clampVertical();
            }
            preva2 = curra2;


            if(gamepad2.x != prevx2){
                U.openClampPos();
            }else{
                U.closeClampPos();
            }
            prevx2 = currx2;


            if(gamepad2.y) {
                U.clampClosedHorizontal();
            }

            U.moveLift(gamepad2.left_stick_y);

            U.moveArm(gamepad2.right_stick_y);


            if(gamepad1.y != prevY){
                U.spinIntake();
            } else {
                U.stopIntake();
            }
            prevY = currY;

            curra1 = gamepad1.a;
            if(curra1 && curra1 != preva1){
                U.toggleRightIntakeServo();
            }
            preva1 = curra1;


            if(gamepad1.b){
                U.closeGrabber();
            }

            if(gamepad1.x){
                U.openGrabber();
            }


            currdpadLeft = gamepad1.dpad_left;
            if(currdpadLeft && currdpadLeft != prevdpadLeft){
                U.moveLeftIntakeServo(true);
            }
            prevdpadLeft = currdpadLeft;


            currdpadRight = gamepad1.dpad_right;
            if(currdpadRight && currdpadRight != prevdpadRight){
                U.moveLeftIntakeServo(false);
            }
            prevdpadRight = currdpadRight;



            // Update Telemetry
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.addData("Lift Input: ", "%5.2f", gamepad2.left_stick_y);
            telemetry.addData("Lift Position: ", "%5d", U.liftMotor.getCurrentPosition());
            telemetry.addData("Arm Input: ", "%5.2f", gamepad2.right_stick_y);
            telemetry.addData("Arm Position: ", "%5d", U.armMotor.getCurrentPosition());
            telemetry.update();
            idle();

        }//end while
    }
}