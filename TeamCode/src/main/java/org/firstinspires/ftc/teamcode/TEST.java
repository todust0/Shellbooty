package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name="TEST", group="Robot")

public class TEST extends LinearOpMode {

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
        waitForStart();}}

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive right for 0.7 seconods
       /* leftFrontDrive.setPower(1);
        rightFrontDrive.setPower(1);
        rightBackDrive.setPower(1);
        leftBackDrive.setPower(1);*/

// Assuming you have a DcMotor named "leftMotor" and an encoder named "leftEncoder"
// Initialize the encoder in your robot's init() method
/*leftEncoder = hardwareMap.get(DcMotorEx.class, "leftMotor").getEncoder();

// Reset the encoder
        leftEncoder.setPosition(0);

// Use the encoder in your autonomous or teleop code
        double targetPosition = 1000; // Set your desired target position
        leftMotor.setTargetPosition((int) targetPosition);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(0.5);

// Check if the motor is still busy reaching the target position
        while (opModeIsActive() && leftMotor.isBusy()) {
        // You can perform other tasks here while the motor is moving
        // For example, update telemetry with motor position
        telemetry.addData("Encoder Position", leftEncoder.getCurrentPosition());
        telemetry.update();
        }

// Once the motor reaches the target position, stop it
        leftMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Set the motor back to normal mode*/
