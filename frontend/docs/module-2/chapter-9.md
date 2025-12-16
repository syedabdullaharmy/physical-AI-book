---
sidebar_position: 9
---

import ChapterToolbar from '@site/src/components/ChapterToolbar';

<ChapterToolbar 
    chapterId="module-2/chapter-9" 
    chapterTitle="Unity for Robot Visualization" 
/>

# Chapter 9: Unity for Robot Visualization

## Introduction

While Gazebo excels at physics simulation, **Unity** provides photorealistic rendering, VR/AR support, and rich user interfaces. This chapter explores Unity-ROS integration for advanced visualization and human-robot interaction.

:::tip Learning Objectives
- Set up Unity with ROS 2
- Create high-fidelity robot visualizations
- Implement VR/AR interfaces
- Build custom control dashboards
- Use Unity ML-Agents for training
:::

## Why Unity for Robotics?

| Feature | Gazebo | Unity |
|---------|--------|-------|
| **Physics** | ✅ Excellent | ⚠️ Good |
| **Graphics** | ⚠️ Basic | ✅ Photorealistic |
| **VR/AR** | ❌ No | ✅ Native support |
| **UI/UX** | ⚠️ Limited | ✅ Rich |
| **ML Training** | ⚠️ Limited | ✅ ML-Agents |

**Use Cases for Unity:**
- Customer demonstrations
- VR teleoperation
- Data visualization dashboards
- Synthetic data generation
- Marketing and presentations

## Unity-ROS 2 Integration

### Installing Unity ROS TCP Connector

```bash
# Install ROS-TCP-Endpoint on ROS 2 side
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint

# Run the endpoint
ros2 run ros_tcp_endpoint default_server_endpoint
```

### Unity Setup

1. **Install Unity Hub** (2021.3 LTS recommended)
2. **Create new 3D project**
3. **Import ROS TCP Connector**
   - Window → Package Manager
   - Add package from git URL: 
     `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`

### Basic Connection

**C# Script:** `ROSConnection.cs`

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using UnityEngine;

public class ROSConnection : MonoBehaviour
{
    private ROSConnection ros;
    
    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.Connect ToROS("192.168.1.100", 10000);
        
        // Subscribe to topic
        ros.Subscribe<StringMsg>("/robot_status", StatusCallback);
        
        // Register publisher
        ros.RegisterPublisher<StringMsg>("/unity_command");
    }
    
    void StatusCallback(StringMsg message)
    {
        Debug.Log($"Received: {message.data}");
    }
    
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            // Publish message
            StringMsg msg = new StringMsg("Command from Unity");
            ros.Publish("/unity_command", msg);
        }
    }
}
```

## Importing Robot Models

### From URDF to Unity

**Method 1: URDF Importer**

1. Install URDF Importer package
2. Assets → Import Robot from URDF
3. Select your `humanoid.urdf` file

**C# Script:** `URDFJointController.cs`

```csharp
using UnityEngine;
using Unity.Robotics.UrdfImporter;

public class URDFJointController : MonoBehaviour
{
    private UrdfRobot robot;
    private ArticulationBody[] joints;
    
    void Start()
    {
        robot = GetComponent<UrdfRobot>();
        joints = GetComponentsInChildren<ArticulationBody>();
        
        Debug.Log($"Found {joints.Length} joints");
    }
    
    public void SetJointPosition(string jointName, float angle)
    {
        foreach (var joint in joints)
        {
            if (joint.name == jointName)
            {
                var drive = joint.xDrive;
                drive.target = angle * Mathf.Rad2Deg;
                joint.xDrive = drive;
            }
        }
    }
}
```

## Subscribing to ROS Topics

### Joint State Visualization

```csharp
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class JointStateSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private ArticulationBody[] joints;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("/joint_states", UpdateJoints);
        
        joints = GetComponentsInChildren<ArticulationBody>();
    }
    
    void UpdateJoints(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            string jointName = msg.name[i];
            float position = (float)msg.position[i];
            
            foreach (var joint in joints)
            {
                if (joint.name == jointName)
                {
                    var drive = joint.xDrive;
                    drive.target = position * Mathf.Rad2Deg;
                    joint.xDrive = drive;
                }
            }
        }
    }
}
```

## Camera Feeds to ROS

### Publishing Camera Images

```csharp
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class CameraPublisher : MonoBehaviour
{
    public Camera robotCamera;
    public string topicName = "/camera/image_raw";
    public int publishRate = 30; // Hz
    
    private ROSConnection ros;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float timer;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
        
        renderTexture = new RenderTexture(640, 480, 24);
        robotCamera.targetTexture = renderTexture;
        texture2D = new Texture2D(640, 480, TextureFormat.RGB24, false);
    }
    
    void Update()
    {
        timer += Time.deltaTime;
        if (timer >= 1f / publishRate)
        {
            timer = 0;
            PublishImage();
        }
    }
    
    void PublishImage()
    {
        // Render camera to texture
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
        texture2D.Apply();
        
        // Convert to ROS message
        ImageMsg msg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                }
            },
            height = 480,
            width = 640,
            encoding = "rgb8",
            step = 640 * 3,
            data = texture2D.GetRawTextureData()
        };
        
        ros.Publish(topicName, msg);
    }
}
```

## VR Integration

### VR Teleoperation

```csharp
using UnityEngine;
using UnityEngine.XR;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VRTeleoperation : MonoBehaviour
{
    public Transform leftController;
    public Transform rightController;
    
    private ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>("/cmd_vel");
        ros.RegisterPublisher<PoseMsg>("/left_hand_pose");
        ros.RegisterPublisher<PoseMsg>("/right_hand_pose");
    }
    
    void Update()
    {
        // Get controller input
        Vector2 leftStick = GetControllerInput(XRNode.LeftHand);
        
        // Publish velocity commands
        TwistMsg vel = new TwistMsg
        {
            linear = new Vector3Msg
            {
                x = leftStick.y,
                y = 0,
                z = 0
            },
            angular = new Vector3Msg
            {
                x = 0,
                y = 0,
                z = leftStick.x
            }
        };
        ros.Publish("/cmd_vel", vel);
        
        // Publish hand poses
        PublishHandPose(leftController, "/left_hand_pose");
        PublishHandPose(rightController, "/right_hand_pose");
    }
    
    Vector2 GetControllerInput(XRNode node)
    {
        InputDevice device = InputDevices.GetDeviceAtXRNode(node);
        Vector2 stick;
        device.TryGetFeatureValue(CommonUsages.primary2DAxis, out stick);
        return stick;
    }
    
    void PublishHandPose(Transform hand, string topic)
    {
        PoseMsg pose = new PoseMsg
        {
            position = new PointMsg
            {
                x = hand.position.x,
                y = hand.position.y,
                z = hand.position.z
            },
            orientation = new QuaternionMsg
            {
                x = hand.rotation.x,
                y = hand.rotation.y,
                z = hand.rotation.z,
                w = hand.rotation.w
            }
        };
        ros.Publish(topic, pose);
    }
}
```

## Custom Dashboards

### Robot Status Dashboard

```csharp
using UnityEngine;
using UnityEngine.UI;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class RobotDashboard : MonoBehaviour
{
    public Text batteryText;
    public Text speedText;
    public Slider batterySlider;
    public Image[] jointHealthIndicators;
    
    private ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<BatteryStateMsg>("/battery", UpdateBattery);
        ros.Subscribe<JointStateMsg>("/joint_states", UpdateJoints);
    }
    
    void UpdateBattery(BatteryStateMsg msg)
    {
        float percentage = msg.percentage * 100;
        batteryText.text = $"Battery: {percentage:F1}%";
        batterySlider.value = msg.percentage;
        
        // Color coding
        if (percentage < 20)
            batterySlider.fillRect.GetComponent<Image>().color = Color.red;
        else if (percentage < 50)
            batterySlider.fillRect.GetComponent<Image>().color = Color.yellow;
        else
            batterySlider.fillRect.GetComponent<Image>().color = Color.green;
    }
    
    void UpdateJoints(JointStateMsg msg)
    {
        for (int i = 0; i < msg.effort.Length && i < jointHealthIndicators.Length; i++)
        {
            float effort = (float)msg.effort[i];
            float maxEffort = 50.0f; // Nm
            
            if (Mathf.Abs(effort) > maxEffort * 0.9f)
                jointHealthIndicators[i].color = Color.red;
            else if (Mathf.Abs(effort) > maxEffort * 0.7f)
                jointHealthIndicators[i].color = Color.yellow;
            else
                jointHealthIndicators[i].color = Color.green;
        }
    }
}
```

## Unity ML-Agents Integration

### Training Robot Behaviors

```csharp
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class HumanoidWalkingAgent : Agent
{
    public Transform target;
    private ArticulationBody[] joints;
    private Rigidbody rBody;
    
    public override void Initialize()
    {
        rBody = GetComponent<Rigidbody>();
        joints = GetComponentsInChildren<ArticulationBody>();
    }
    
    public override void OnEpisodeBegin()
    {
        // Reset position
        transform.localPosition = new Vector3(0, 1, 0);
        transform.localRotation = Quaternion.identity;
        rBody.velocity = Vector3.zero;
        rBody.angularVelocity = Vector3.zero;
        
        // Randomize target
        target.localPosition = new Vector3(
            Random.Range(-5f, 5f),
            0.5f,
            Random.Range(-5f, 5f)
        );
    }
    
    public override void CollectObservations(VectorSensor sensor)
    {
        // Target position relative to agent
        sensor.AddObservation(target.localPosition - transform.localPosition);
        
        // Agent velocity
        sensor.AddObservation(rBody.velocity);
        sensor.AddObservation(rBody.angularVelocity);
        
        // Joint positions and velocities
        foreach (var joint in joints)
        {
            sensor.AddObservation(joint.jointPosition);
            sensor.AddObservation(joint.jointVelocity);
        }
    }
    
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Apply actions to joints
        for (int i = 0; i < joints.Length; i++)
        {
            var drive = joints[i].xDrive;
            drive.target = actions.ContinuousActions[i] * 180f; // ±180°
            joints[i].xDrive = drive;
        }
        
        // Calculate reward
        float distanceToTarget = Vector3.Distance(
            transform.localPosition,
            target.localPosition
        );
        
        // Reward for moving toward target
        AddReward(-distanceToTarget * 0.01f);
        
        // Penalty for falling
        if (transform.localPosition.y < 0.5f)
        {
            AddReward(-1.0f);
            EndEpisode();
        }
        
        // Reward for reaching target
        if (distanceToTarget < 1.0f)
        {
            AddReward(1.0f);
            EndEpisode();
        }
    }
    
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control for testing
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
    }
}
```

## Lighting and Post-Processing

### Photorealistic Rendering

```csharp
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class GraphicsQuality : MonoBehaviour
{
    public Volume postProcessVolume;
    
    void Start()
    {
        // Enable HDR
        Camera.main.allowHDR = true;
        
        // Configure post-processing
        if (postProcessVolume.profile.TryGet<Bloom>(out var bloom))
        {
            bloom.intensity.value = 0.3f;
            bloom.threshold.value = 1.0f;
        }
        
        if (postProcessVolume.profile.TryGet<DepthOfField>(out var dof))
        {
            dof.mode.value = DepthOfFieldMode.Bokeh;
            dof.focusDistance.value = 5.0f;
        }
        
        // Set quality
        QualitySettings.shadows = ShadowQuality.All;
        QualitySettings.shadowResolution = ShadowResolution.VeryHigh;
    }
}
```

## Performance Optimization

### LOD (Level of Detail)

```csharp
using UnityEngine;

public class LODSetup : MonoBehaviour
{
    void Start()
    {
        LODGroup lodGroup = gameObject.AddComponent<LODGroup>();
        
        LOD[] lods = new LOD[3];
        
        // High detail (0-50%)
        lods[0] = new LOD(0.5f, GetRenderers("HighDetail"));
        
        // Medium detail (50-20%)
        lods[1] = new LOD(0.2f, GetRenderers("MediumDetail"));
        
        // Low detail (20-5%)
        lods[2] = new LOD(0.05f, GetRenderers("LowDetail"));
        
        lodGroup.SetLODs(lods);
        lodGroup.RecalculateBounds();
    }
    
    Renderer[] GetRenderers(string tag)
    {
        GameObject[] objs = GameObject.FindGameObjectsWithTag(tag);
        Renderer[] renderers = new Renderer[objs.Length];
        for (int i = 0; i < objs.Length; i++)
        {
            renderers[i] = objs[i].GetComponent<Renderer>();
        }
        return renderers;
    }
}
```

## Summary

✅ Unity-ROS 2 integration  
✅ URDF import and visualization  
✅ VR/AR teleoperation  
✅ Custom dashboards  
✅ ML-Agents for training  
✅ Photorealistic rendering  

## Practice Exercises

1. Import your humanoid URDF into Unity
2. Create VR teleoperation interface
3. Build custom status dashboard
4. Train walking behavior with ML-Agents

## Next Chapter

Module 3 begins! In **Chapter 10**, we'll explore:
- NVIDIA Isaac Sim and SDK
- RTX ray tracing
- Synthetic data generation

👉 [Continue to Module 3: Chapter 10 →](../module-3/chapter-10.md)

---

:::tip Module 2 Complete!
You now understand simulation, physics, sensors, and visualization. Ready for AI-powered robotics!
:::
