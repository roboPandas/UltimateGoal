package org.firstinspires.ftc.teamcode.ultimategoal.scrimmage.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ultimategoal.scrimmage.Util.MecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.ultimategoal.scrimmage.Util.PoseStorage;
import org.firstinspires.ftc.teamcode.ultimategoal.vision.RingStackDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Autonomous(group = "advanced")
public class RedSideOneOpenCV extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TURN_1,         // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        WAIT_1,         // Then we're gonna wait a second
        TURN_2,         // Finally, we're gonna turn again
        IDLE            // Our bot will enter the IDLE state when done
    }

    private OpenCvCamera webcam;
    StackDeterminationPipeline pipeline;

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2d startPose = new Pose2d(15, 10, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize computer vision
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        pipeline = new StackDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // Initialize our lift
        Lift lift = new Lift(hardwareMap);

        // Initialize SampleMecanumDrive
        MecanumDriveCancelable drive = new MecanumDriveCancelable(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(45, -20), Math.toRadians(90))
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineTo(new Vector2d(45, 0))
                .build();

        // Define the angle to turn at
        double turnAngle1 = Math.toRadians(-270);

        // Third trajectory
        // We have to define a new end pose because we can't just call trajectory2.end()
        // Since there was a point turn before that
        // So we just take the pose from trajectory2.end(), add the previous turn angle to it
        Pose2d newLastPose = trajectory2.end().plus(new Pose2d(0, 0, turnAngle1));
        Trajectory trajectory3 = drive.trajectoryBuilder(newLastPose)
                .lineToConstantHeading(new Vector2d(-15, 0))
                .build();

        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Define the angle for turn 2
        double turnAngle2 = Math.toRadians(720);

        waitForStart();

        if (isStopRequested()) return;

        //Separating the paths based on the computer vision
            if (pipeline.position == StackDeterminationPipeline.RingPosition.FOUR){
                //Code for 4 rings
            }
            else if (pipeline.position == StackDeterminationPipeline.RingPosition.ONE){
                //Code for 1 rings
            }
            else {
                //Code for no rings
            }

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case TRAJECTORY_2:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TURN_1;
                        drive.turnAsync(turnAngle1);
                    }
                    break;
                case TURN_1:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectoryAsync(trajectory3);
                    }
                    break;
                case TRAJECTORY_3:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.TURN_2;
                        drive.turnAsync(turnAngle2);
                    }
                    break;
                case TURN_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose and ring stack to telemetry
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware
        }

        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift
        }
    }

    //Robot needs to process the image to determine stack
    public static class StackDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the ring position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}
