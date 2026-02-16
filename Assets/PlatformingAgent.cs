using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class PlatformingAgent : Agent
{
    [Header("References")]
    public Transform targetTransform;
    public Transform checkpointParent;
    private Rigidbody rb;

    [Header("Movement Settings")]
    public float rollForce = 25f;
    public float jumpForce = 10f;
    [Range(0, 1)] public float airControlMultiplier = 0.3f;

    [Header("Ground Check")]
    public LayerMask groundLayer;
    public float groundCheckDistance = 0.6f;
    
    private bool isGrounded;
    private float jumpCooldown = 0f;
    private float lastMaxHeight;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        // Set collision detection to continuous for better jumping accuracy
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
    }

    public override void OnEpisodeBegin()
    {
        // 1. Reset Physics
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        
        // 2. Reset Position
        transform.localPosition = new Vector3(0, 0.5f, 0);
        
        // 3. Reset Height Tracking
        lastMaxHeight = transform.localPosition.y;

        // 4. Reactivate all Checkpoints
        if (checkpointParent != null)
        {
            foreach (Transform child in checkpointParent)
            {
                child.gameObject.SetActive(true);
            }
        }
    }

    private void Update()
    {
        // Handle Jump Cooldown logic to prevent double-jumping
        if (jumpCooldown > 0)
        {
            jumpCooldown -= Time.deltaTime;
            isGrounded = false;
        }
        else
        {
            // Raycast down to see if we are on a platform/floor
            isGrounded = Physics.Raycast(transform.position, Vector3.down, groundCheckDistance, groundLayer);
        }

        // Debug visual: Green = Grounded, Red = Air
        Debug.DrawRay(transform.position, Vector3.down * groundCheckDistance, isGrounded ? Color.green : Color.red);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Observation Count: 8
        // 1-3: Relative position to target
        sensor.AddObservation(targetTransform.localPosition - transform.localPosition);
        
        // 4-6: Current linear velocity
        sensor.AddObservation(rb.linearVelocity);
        
        // 7: Grounded state (1 or 0)
        sensor.AddObservation(isGrounded ? 1.0f : 0.0f);
        
        // 8: Specific height difference to target
        sensor.AddObservation(targetTransform.localPosition.y - transform.localPosition.y);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // 1. Get Action Values
        int moveAction = actions.DiscreteActions[0];
        int jumpAction = actions.DiscreteActions[1];

        // 2. Movement Logic (Translate numbers to direction)
        Vector3 forceDir = Vector3.zero;
        if (moveAction == 1) forceDir = Vector3.forward;
        else if (moveAction == 2) forceDir = Vector3.back;
        else if (moveAction == 3) forceDir = Vector3.right;
        else if (moveAction == 4) forceDir = Vector3.left;

        // Apply Rolling Force (apply airControlMultiplier if not grounded)
        float currentForce = isGrounded ? rollForce : rollForce * airControlMultiplier;
        rb.AddForce(forceDir * currentForce);

        // 3. Jumping Logic
        if (jumpAction == 1 && isGrounded)
        {
            rb.AddForce(Vector3.up * jumpForce, ForceMode.Impulse);
            
            // Use the cooldown we created to prevent micro-double-jumps
            jumpCooldown = 0.15f; 
            isGrounded = false;
        }

        // --- REWARDS & PENALTIES ---

        // 4. Existence Penalty (Encourages finishing the task quickly)
        // At -0.001 per step, if Max Step is 5000, the worst penalty is -5.0.
        AddReward(-0.001f);

        // 5. Dynamic Height Reward
        float currentHeight = transform.localPosition.y;

        if (currentHeight > lastMaxHeight)
        {
            // Reward the agent for new progress (climbing higher than ever before in this episode)
            float heightGain = currentHeight - lastMaxHeight;
            AddReward(heightGain * 0.1f); // Adjust this multiplier if progress is too slow
            lastMaxHeight = currentHeight;
        }
        else if (currentHeight < lastMaxHeight - 1.5f)
        {
            // If the agent falls significantly, reset the MaxHeight tracker.
            // This is key: It allows the agent to "re-earn" height rewards 
            // by climbing back up, rather than giving up.
            lastMaxHeight = currentHeight;
        }

        // 6. Termination: Falling off the world
        // If the agent falls below the ground level (-1.0), end the episode.
        if (currentHeight < -1.0f)
        {
            AddReward(-1.0f); // Significant penalty for falling completely off
            EndEpisode();
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Checkpoint"))
        {
            AddReward(0.5f); // Large reward for reaching a new platform
            other.gameObject.SetActive(false); // Disable so it's not multi-triggered
            Debug.Log("Checkpoint Reached!");
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Target"))
        {
            AddReward(2.0f); // Massive win reward
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActions = actionsOut.DiscreteActions;
        
        // Branch 0: WASD
        discreteActions[0] = 0;
        if (Input.GetKey(KeyCode.W)) discreteActions[0] = 1;
        else if (Input.GetKey(KeyCode.S)) discreteActions[0] = 2;
        else if (Input.GetKey(KeyCode.D)) discreteActions[0] = 3;
        else if (Input.GetKey(KeyCode.A)) discreteActions[0] = 4;

        // Branch 1: Space to Jump
        discreteActions[1] = Input.GetKey(KeyCode.Space) ? 1 : 0;
    }
}