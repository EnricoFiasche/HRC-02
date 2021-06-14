using JointState = RosMessageTypes.Sensor.JointState;
using RosMessageTypes.HumanBaxterCollaboration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using UnityEngine;

public class ROSInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;
    
    // Variables required for ROS communication
    public string trajectoryTopicName = "baxter_moveit_trajectory";
    public string unityTfTopicName = "unity_tf";
    public string jointStateTopicName = "baxter_joint_states";
    public string gripperTopicName = "baxter_gripper";
    public string restTopicName = "baxter_rest_position";
    public string leftGripperTopicName = "baxter_left_gripper";
    public string rightGripperTopicName = "baxter_right_gripper";
    public string stopTopicName = "baxter_stop";    

    public GameObject baxter;
    public GameObject avatar;

    private BaxterController controller;
    private TFManager tfManager;

    private bool simStarted;
    private float timeElapsedTf;
    private float timeElapsedJS;
    private float timeElapsedGP;
    private float publishTfFrequency;
    private float publishJSFrequency;
    private float publishGPFrequency;

    void Start()
    {
        simStarted = false;
        timeElapsedTf = 0;
        timeElapsedJS = 0;
        timeElapsedGP= 0;
        publishTfFrequency = 0.05f;
        publishJSFrequency = 1.0f;
        publishGPFrequency = 0.5f;

        // Get ROS connection static instance
        ros = ROSConnection.instance;

        // Instantiate Baxter Controller
        controller = gameObject.AddComponent<BaxterController>();
        controller.Init(baxter);

        // Subscribe to MoveIt trajectory topic
        ros.Subscribe<BaxterTrajectory>(trajectoryTopicName, controller.TrajectoryResponse);
	ros.Subscribe<Bool>(gripperTopicName, controller.GripperResponse);
	ros.Subscribe<String>(restTopicName, controller.RestResponse);
        ros.Subscribe<Bool>(stopTopicName, controller.StopResponse);
	
        // Instantiate TF Manager component
        tfManager = gameObject.AddComponent<TFManager>();
        tfManager.Init(avatar, baxter.transform.Find("ground"));

    }

    public void RestCommand()
    {
        controller.GoToRestPosition("both");
        simStarted = true;
    }

    public void Update()
    {
        if (simStarted)
        {
            timeElapsedTf += UnityEngine.Time.deltaTime;
            timeElapsedJS += UnityEngine.Time.deltaTime;
            timeElapsedGP += UnityEngine.Time.deltaTime;

            if (timeElapsedTf > publishTfFrequency)
            {
                UnityTf unityTfMsg = tfManager.GetUnityTfMessage();
                ros.Send(unityTfTopicName, unityTfMsg);

                timeElapsedTf = 0;
            }

            if (timeElapsedJS > publishJSFrequency)
            {
                JointState jointStateMsg = controller.GetBaxterJointState();
                ros.Send(jointStateTopicName, jointStateMsg);

                timeElapsedJS = 0;
            }

            if (timeElapsedGP > publishGPFrequency)
            {
                PoseStamped leftGripper = controller.GetLeftGripper();
		PoseStamped rightGripper = controller.GetRightGripper();

                ros.Send(leftGripperTopicName, leftGripper);
                ros.Send(rightGripperTopicName, rightGripper);

                timeElapsedGP = 0;
            }
        }
    }
}