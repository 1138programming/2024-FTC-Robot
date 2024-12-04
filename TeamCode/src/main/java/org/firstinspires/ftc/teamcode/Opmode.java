/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="OpMode done 2", group="Linear OpMode")

public class Opmode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx armMotor = null;
    private DcMotor teleMotor = null;

    private Servo Wrist = null;
    private Servo Roller = null;

    private    IntegratingGyroscope gyro;
    private NavxMicroNavigationSensor navxMicro;
    private boolean start = false;
    private boolean reversed  = false;
    private boolean lastpress = false;
    private final double NEW_P = 25;
    private final double NEW_I = 0;
    private final double NEW_D = 3.6;//3.5
    private final double NEW_F = 0;
    PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
    private int armTarget= 0;

    public double speedlimiter(double input) {
        double output =0;
        double limitedOutput = -Math.log10(-(Math.abs(input)-1.1));
        if (limitedOutput <= 0.1) {
            output = 0.1;
        }
        else {
            output = limitedOutput;
        }
        if (input < 0) {
            output *=-1;
        }
        return output;
    }
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "Left_Front");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "Left_Back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_Front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Right_Back");

        armMotor = hardwareMap.get(DcMotorEx.class, "Arm");
        teleMotor = hardwareMap.get(DcMotor.class, "tele");
        armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Roller = hardwareMap.get(Servo.class, "Roller");

        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro;

        armTarget = armMotor.getCurrentPosition() + 100;

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        Wrist.setDirection(Servo.Direction.FORWARD);
        Roller.setDirection(Servo.Direction.FORWARD);
        teleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        // A timer helps provide feedback while calibration is taking place
        ElapsedTime timer = new ElapsedTime();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double wristposition = 0;

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;



            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double lowSpeed = 0.45;
            double midSpeed = 0.70;
            double highSpeed = 0.90;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = (axial + lateral + yaw) * midSpeed;
            double rightFrontPower = (axial - lateral - yaw) * midSpeed;
            double leftBackPower   = (axial - lateral + yaw) * midSpeed;
            double rightBackPower  = (axial + lateral - yaw) * midSpeed;



            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            if (!reversed) {
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }
            else {
                leftFrontDrive.setPower(-leftFrontPower);
                rightFrontDrive.setPower(-rightFrontPower);
                leftBackDrive.setPower(-leftBackPower);
                rightBackDrive.setPower(-rightBackPower);
            }

            if (gamepad1.y && !lastpress){
                reversed = !reversed;
                lastpress = true;
            }
            else if (!gamepad1.y){
                lastpress = false;
            }





            if (gamepad2.left_bumper) {
//                armMotor.setPower(1);
                armTarget = armMotor.getCurrentPosition() + 50;
            }
            else if (gamepad2.left_trigger > 0) {
//                armMotor.setPower(-1);
                armTarget = armMotor.getCurrentPosition() - 50;
            }
//            else {
////                armTarget = armMotor.getCurrentPosition();
////                armMotor.setPower(0);
//            }


            armMotor.setTargetPosition(armTarget);
            armMotor.setPower(1);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);






            if (gamepad2.dpad_down) {
                teleMotor.setPower(0.7);
            }
            else if (gamepad2.dpad_up && teleMotor.getCurrentPosition() > -1920) {
                teleMotor.setPower(-0.7);
            }
            else {
                teleMotor.setPower(0);
            }








            if (gamepad2.right_bumper) {
                start = true;
                Wrist.setPosition(0.15);
            }
            else if (gamepad2.right_trigger > 0) {
                start = true;
                Wrist.setPosition(0.85);
            }
            else  if (start){
                Wrist.setPosition(0.5);
            }
            else {
//                Wrist.setPosition(0.85);
                 Wrist.setPosition(0.5);

            }


            if (gamepad2.a) {
                Roller.setPosition(0.1);
            }
            else if (gamepad2.b) {
                Roller.setPosition(0.9);
            }
            else {
                Roller.setPosition(0.5);
            }





            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Roller Pos",  Roller.getPosition());
            telemetry.addData("Wrist Pos",  Wrist.getPosition());
            telemetry.addData("Arm Pos",  armMotor.getCurrentPosition());
            telemetry.addData("Arm Pid", armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.addData("Tele Pos",  teleMotor.getCurrentPosition());
            telemetry.addData("Arm Vel", armMotor.getVelocity());
            telemetry.addData("Arm target", armMotor.getTargetPosition());
            telemetry.addData("target",  armTarget);
            telemetry.addData("Gyro",  gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Gyro",  gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
            telemetry.addData("Gyro",  gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.addData("Reversed:",reversed);
            telemetry.update();
        }
    }}