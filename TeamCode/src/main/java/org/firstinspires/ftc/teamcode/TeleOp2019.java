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

    boolean prevY = false;
    boolean currY;

    boolean currdpadLeft;
    boolean prevdpadLeft = false;

    boolean currdpadRight;
    boolean prevdpadRight = false;

    boolean curra1;
    boolean preva1 = false;

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
            //right = gamepad1.left_stick_x;
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

/////////////////////////////////////////////////////////////////////////////
            if(gamepad2.b){
                U.clampOpenHorizontal();
            }

            if(gamepad2.a){
                U.closeClamp();
            }

            if(gamepad2.x){
                U.grabBlock();
            }

            if(gamepad2.y) {
                U.clampClosedHorizontal();
            }

            U.moveLift(gamepad2.left_stick_y);

            U.moveArm(gamepad2.right_stick_y);
////////////////////////////////////////////////////////////////////////////////////////
            if(gamepad1.y != prevY){
                U.spinIntake();
            }else{
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

            //------------------------------------------
            //-------------------------------------------



            // Update Telemetry
            telemetry.addData(">", "Press Stop to end test.");

            if (gamepad1.a) {
                telemetry.addLine("Button A is pressed on pad 1");
            } else if (gamepad1.b) {
                telemetry.addLine("Button B is pressed on pad 1");
            } else {
                telemetry.addLine("Neither button is pressed on pad 1");
            }

            if (gamepad2.a) {
                telemetry.addLine("Button A is pressed on pad 2");
            } else if (gamepad2.b) {
                telemetry.addLine("Button B is pressed on pad 2");
            } else {
                telemetry.addLine("Neither button is pressed on pad 2");
            }

            telemetry.update();
            idle();

            telemetry.addLine();




        }//end while
    }
}