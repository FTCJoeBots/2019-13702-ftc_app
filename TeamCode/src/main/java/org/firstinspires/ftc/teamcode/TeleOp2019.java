package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 *import com.qualcomm.robotcore.hardware.DcMotor;
 *
 *
 */

/**
 *Notes For this TeleOp Code. This code is for Comp and all proggramers should review over this
 *code and understand this code for the possibility that a question may be asked related to TeleOp and
 *you should be able to explain in good detail everything in this code.
 *11/16/17-> Changed all gamepad's in code to correct gamepad (i.e some gamepad1's to gamepad2)
 ***11/18/17-> Competition Notes below
 *Notes-> Autonomous is incorrect, Not much was wrong from a software sandpoint but hardware issues were fixed
 *Autonomous issues included: Incorrect spinning causing us to move out of destination,
 *To much time on the down motion of the clamp and arm.
 *These issues are still not resolved
 * Recomendation for autonomous issues(Not Offical):Fine tune the timer on the clamp
 * Fine tune the movements and LOWER the TIME OF MOVEMENT in autonomous.
 * List of issues at Comp(1)-> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1r_liipKBU7GHfONdxq9E6d4f7zikcCuXwDL2bsQfwm0/edit?usp=sharing
 *G-Sheet of time VS Heading for autonomous -> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1pqv0iN94fFd5KvX1YIWP7z39HgpURXsscn0zPujs1q4/edit?usp=sharing
*/
@TeleOp(name="13702TeleOp", group="TeleOp")

public class TeleOp2019 extends LinearOpMode {

    double forward;
    double clockwise;
    double right;
    double k;
    double power0;
    double power1;
    double power2;
    double power3;
    double max;

    boolean prevy1 = false;
    boolean curry1;

    boolean prevb1 = false;
    boolean currb1;

    boolean prevx1 = false;
    boolean currx1;

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

    boolean currrbumper1;
    boolean prevrbumper1 = false;

    boolean currlbumper1;
    boolean prevlbumper1 = false;

    boolean isClampOpen = false;

    double clampCurr;

    boolean clampMoveDone = false;

    int liftTarget = 0;

    HardwareJoeBot2019 robot = new HardwareJoeBot2019();
    Utility13702 U = new Utility13702();

    @Override
    public void runOpMode() throws InterruptedException {

        U.init(hardwareMap, this);
        robot.init(hardwareMap, this);

        waitForStart();

        //start of loop
        while (opModeIsActive()) {


            if(gamepad1.right_bumper) {
           /* telemetry.addData("rotClampServo:", U.rotClampServo.getPosition());
            telemetry.addData("ClampServo:", U.clampServo.getPosition());
            telemetry.update(); */

                //Drive Via "Analog Sticks" (Not Toggle)
                //Set initial motion parameters to Gamepad1 Inputs
                forward = -gamepad1.left_stick_y;
                //right = gamepad1.left_stick_x;
                right = -gamepad1.left_trigger + gamepad1.right_trigger;
                clockwise = gamepad1.right_stick_x;

                // Add a tuning constant "K" to tune rotate axis sensitivity
                k = .6;
                clockwise = clockwise * k; //Make sure the "= Clockwise" is "= -clockwise"


                // Calculate motor power
                power0 = 0.3 * (forward + clockwise + right);
                power1 = 0.3 * (forward - clockwise - right);
                power2 = 0.3 * (forward + clockwise - right);
                power3 = 0.3 * (forward - clockwise + right);

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
            } else {
                /* telemetry.addData("rotClampServo:", U.rotClampServo.getPosition());
            telemetry.addData("ClampServo:", U.clampServo.getPosition());
            telemetry.update(); */

                //Drive Via "Analog Sticks" (Not Toggle)
                //Set initial motion parameters to Gamepad1 Inputs
                forward = -gamepad1.left_stick_y;
                //right = gamepad1.left_stick_x;
                right = -gamepad1.left_trigger + gamepad1.right_trigger;
                clockwise = gamepad1.right_stick_x;

                // Add a tuning constant "K" to tune rotate axis sensitivity
                k = .6;
                clockwise = clockwise * k; //Make sure the "= Clockwise" is "= -clockwise"


                // Calculate motor power
                power0 = (forward + clockwise + right);
                power1 = (forward - clockwise - right);
                power2 = (forward + clockwise - right);
                power3 = (forward - clockwise + right);

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
            }


////////////////////////////////////////////////////////////////////////////

            while(gamepad2.y){
               //arm motor position
                U.moveArmEncoder(U.ARM_DEFAULT_POSITION);

                //lift motor position
                U.moveLiftEncoder(U.LIFT_DEFAULT_POSITION);
            }


            curra2 = gamepad2.a;

            if (curra2 && curra2 != preva2) {
                if (isClampOpen == false) {
                    U.clampDirectionToggle();
                } else {
                    U.clampSlantedDirectionToggle();
                }
            }
            preva2 = curra2;


            if (gamepad2.x != prevx2) {
                U.openClampPos();

                isClampOpen = true;
                clampMoveDone = false;

                if (clampMoveDone == false) {
                    if (U.isClampVertical){
                        U.clampVerticalSlanted();
                    }
                    }else if(U.isClampVertical == false){
                        U.clampHorizontalSlanted();
                    }
                clampMoveDone = true;

            } else {
                U.closeClampPos();

                isClampOpen = false;
                clampMoveDone = false;

                if (clampMoveDone == false) {
                    if (U.isClampVertical){
                        U.clampVertical();
                    }
                }else if(U.isClampVertical == false){
                    U.clampHorizontalPos();
                }
                clampMoveDone = true;


            }
            clampMoveDone = false;

            U.moveLiftStick(gamepad2.left_stick_y);
            //telemetry.addData("pos",U.liftMotor.getCurrentPosition());
            //telemetry.addData("target: ", U.liftTarget);

            U.moveArm(gamepad2.right_stick_y);

            U.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("armpos",U.armMotor.getCurrentPosition());
            telemetry.addData("armtarget: ", U.armTarget);

            telemetry.addData("armpos",U.liftMotor.getCurrentPosition());
            telemetry.addData("armtarget: ", U.liftTarget);

////////////////////////////////////////////////////////////////////////////////////////
            if (gamepad1.y != prevy1) {
                U.spinIntake();
            } else {
                U.stopIntake();
            }
            prevy1 = curry1;


            if (gamepad1.b != prevb1) {
                U.reverseIntake();
            } else {
                U.stopIntake();
            }
            prevb1 = currb1;


            curra1 = gamepad1.a;
            if (curra1 && curra1 != preva1) {
                U.toggleRightIntakeServo();
            }
            preva1 = curra1;


            currx1 = gamepad1.x;
            if (currx1 && currx1 != prevx1) {
                U.toggleGrabber();
            }
            prevx1 = currx1;


            currdpadLeft = gamepad1.dpad_left;
            if (currdpadLeft && currdpadLeft != prevdpadLeft) {
                U.moveLeftIntakeServo(true);
            }
            prevdpadLeft = currdpadLeft;

            currdpadRight = gamepad1.dpad_right;
            if (currdpadRight && currdpadRight != prevdpadRight) {
                U.moveLeftIntakeServo(false);
            }
            prevdpadRight = currdpadRight;

            if(gamepad1.left_bumper){
                U.leftIntakeServoOut();
                U.leftIntakeServoCurr = U.LEFT_INTAKE_SERVO_OUT_POSITION;
            }

            //------------------------------------------
            //-------------------------------------------


            // Update Telemetry
            telemetry.addData(">", "Press Stop to end test.");

            telemetry.update();
            idle();



        }//end while
    }
}
