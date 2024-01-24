/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="RedFrontMid", group="Robot")

public class RedFrontMid extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftBackDrive = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor ArmMotor1 = null;
    private DcMotor ArmMotor2 = null;
    private double ArmPower = 0.2;
    private double speed = 0.75;


    static final double FORWARD_SPEED = 0.35;
    static final double TURN_SPEED = 0.5;
    static final double BACKWARD_SPEED = -0.35;


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        ArmMotor1 = hardwareMap.get(DcMotor.class, "MotorArm1");
        ArmMotor2 = hardwareMap.get(DcMotor.class, "MotorArm2");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        ArmMotor1.setDirection(DcMotor.Direction.FORWARD);
        ArmMotor2.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Shellbooty", "Big");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive right for 0.7 seconods
       /* leftFrontDrive.setPower(1);
        rightFrontDrive.setPower(1);
        rightBackDrive.setPower(1);
        leftBackDrive.setPower(1);*/

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1.7 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            leftFrontDrive.setPower(FORWARD_SPEED);
            rightFrontDrive.setPower(FORWARD_SPEED);
            rightBackDrive.setPower(BACKWARD_SPEED);
            leftBackDrive.setPower(BACKWARD_SPEED);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <0.5 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            leftFrontDrive.setPower(BACKWARD_SPEED);
            rightFrontDrive.setPower(BACKWARD_SPEED);
            rightBackDrive.setPower(FORWARD_SPEED);
            leftBackDrive.setPower(FORWARD_SPEED);

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <0.89 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            leftFrontDrive.setPower(-FORWARD_SPEED);
            rightFrontDrive.setPower(-BACKWARD_SPEED);
            rightBackDrive.setPower(-FORWARD_SPEED);
            leftBackDrive.setPower(-BACKWARD_SPEED);


        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <0.34 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            leftFrontDrive.setPower(-BACKWARD_SPEED);
            rightFrontDrive.setPower(-FORWARD_SPEED);
            rightBackDrive.setPower(-FORWARD_SPEED);
            leftBackDrive.setPower(-BACKWARD_SPEED);

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1.9 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            leftFrontDrive.setPower(BACKWARD_SPEED);
            rightFrontDrive.setPower(BACKWARD_SPEED);
            rightBackDrive.setPower(FORWARD_SPEED);
            leftBackDrive.setPower(FORWARD_SPEED);

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();


        while (opModeIsActive() && (runtime.seconds() <1 )) {}
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1.7 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            ArmMotor2.setPower(-0.4);

        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <0.3 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            leftFrontDrive.setPower(FORWARD_SPEED);
            rightFrontDrive.setPower(FORWARD_SPEED);
            rightBackDrive.setPower(BACKWARD_SPEED);
            leftBackDrive.setPower(BACKWARD_SPEED);

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() <1 )) {}
        runtime.reset();
        ArmMotor2.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            ArmMotor2.setPower(0.5);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <2)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            leftFrontDrive.setPower(-BACKWARD_SPEED);
            rightFrontDrive.setPower(-FORWARD_SPEED);
            rightBackDrive.setPower(-FORWARD_SPEED);
            leftBackDrive.setPower(-BACKWARD_SPEED);

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() <0.5 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            leftFrontDrive.setPower(BACKWARD_SPEED);
            rightFrontDrive.setPower(BACKWARD_SPEED);
            rightBackDrive.setPower(FORWARD_SPEED);
            leftBackDrive.setPower(FORWARD_SPEED);
        }}}

