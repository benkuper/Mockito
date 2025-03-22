using System;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class HandleSystem
{
    public Transform handle;
    public Transform[] pulleys;
    public LineRenderer wire;

    public float initialHandlePulleyLength;
    public float initialPulleyTargetLength;
    public float currentTargetPulleyLength;

}

[ExecuteInEditMode]
public class PulleySystem : MonoBehaviour
{
    //this script is running on the target object

    [SerializeField]
    public List<HandleSystem> handleSystems;

    public bool editMode;
    bool _editMode;

    public int maxIterations = 100;
    [Range(0.0001f, 0.01f)]
    public float threshold = 0.001f;

    public float error = 0.0f;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void OnEnable()
    {
        calculateWireLength();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.E) && Input.GetKey(KeyCode.LeftControl))
        {
            editMode = !editMode;
        }

        updateEditMode();

        if (!editMode)
        {
            updateLengths();
            if (handleSystems.Count == 2)
            {
                Vector3? pos = SolveForTwoPoints(handleSystems[0], handleSystems[1]);
                transform.position = pos ?? transform.position;
            }
            else if (handleSystems.Count >= 3)
            {
                transform.position = SolveWeightedMultilateration();
            }
        }
        updateWirePositions();
    }

    void updateEditMode()
    {
        if (editMode != _editMode)
        {
            _editMode = editMode;

            calculateWireLength();
        }
    }

    void calculateWireLength()
    {
        foreach (var handleSystem in handleSystems)
        {
            handleSystem.initialHandlePulleyLength = getHandlePulleyDistance(handleSystem);
            handleSystem.initialPulleyTargetLength = Vector3.Distance(getLastPulleyPosition(handleSystem), transform.position);
        }
    }

    float getHandlePulleyDistance(HandleSystem handleSystem)
    {
        float result = 0;
        if (handleSystem.pulleys.Length > 0)
        {
            result = Vector3.Distance(handleSystem.handle.position, handleSystem.pulleys[0].position);
            for (int i = 0; i < handleSystem.pulleys.Length - 1; i++)
            {
                result += Vector3.Distance(handleSystem.pulleys[i].position, handleSystem.pulleys[i + 1].position);
            }
        }

        return result;
    }

    Vector3 getLastPulleyPosition(HandleSystem hs)
    {
        return hs.pulleys[hs.pulleys.Length - 1].position;
    }


    void updateLengths()
    {
        foreach (var handleSystem in handleSystems)
        {
            float curHandlePulleyLength = getHandlePulleyDistance(handleSystem);
            float diffHandlePulley = curHandlePulleyLength - handleSystem.initialHandlePulleyLength;
            handleSystem.currentTargetPulleyLength = handleSystem.initialPulleyTargetLength - diffHandlePulley;
            //Debug.Log(handleSystem.initialPulleyTargetLength + " / " + diffHandlePulley + " /" + handleSystem.currentTargetPulleyLength);
        }
    }

    void updateWirePositions()
    {
        foreach (var handleSystem in handleSystems)
        {
            LineRenderer lr = handleSystem.wire.GetComponent<LineRenderer>();
            Vector3[] positions = new Vector3[handleSystem.pulleys.Length + 2];
            positions[0] = handleSystem.handle.position;
            for (int i = 0; i < handleSystem.pulleys.Length; i++)
            {
                positions[i + 1] = handleSystem.pulleys[i].position;
            }
            positions[positions.Length - 1] = transform.position;
            lr.positionCount = positions.Length;
            lr.SetPositions(positions);
        }
    }

    public Vector3? SolveForTwoPoints(HandleSystem s1, HandleSystem s2)
    {
        Vector3 p1 = getLastPulleyPosition(s1);
        Vector3 p2 = getLastPulleyPosition(s2);
        float d1 = s1.currentTargetPulleyLength; // Assuming the first pulley for simplicity
        float d2 = s2.currentTargetPulleyLength; // Assuming the first pulley for simplicity

        Vector3 dir = (p2 - p1).normalized;
        float dist = Vector3.Distance(p1, p2);

        // Check solvability
        if (dist > d1 + d2 || dist < Mathf.Abs(d1 - d2))
        {
            Debug.LogWarning("No solution exists (spheres don't intersect).");
            return null; // Or return the closest approximation if preferred
        }

        // Intersection circle parameters
        float a = (d1 * d1 - d2 * d2 + dist * dist) / (2 * dist);
        float h = Mathf.Sqrt(d1 * d1 - a * a);
        Vector3 circleCenter = p1 + a * dir;
        Vector3 circlePlaneNormal = dir;

        // To get the lowest point, project gravity onto circle plane
        Vector3 gravityProjected = Vector3.ProjectOnPlane(Vector3.down, circlePlaneNormal).normalized;

        // Lowest point on circle
        Vector3 lowestPoint = circleCenter + gravityProjected * h;

        return lowestPoint;
    }

    public Vector3 SolveWeightedMultilateration()
    {
        List<Vector3> anchors = new List<Vector3>();
        List<float> distances = new List<float>();

        Vector3 gravityDir = Vector3.down;

        foreach (var hs in handleSystems)
        {
            anchors.Add(getLastPulleyPosition(hs));
            distances.Add(hs.currentTargetPulleyLength);
        }

        // Initial guess: below the centroid to prioritize lowest position
        Vector3 guess = Vector3.zero;
        foreach (var p in anchors) guess += p;
        guess /= anchors.Count;
        guess += gravityDir.normalized; // bias slightly downwards initially

        float stepSize = 0.1f;
        Vector3 bestGuess = guess;
        float bestError = float.MaxValue;

        for (int iter = 0; iter < maxIterations; iter++)
        {
            Vector3 gradient = Vector3.zero;
            float totalError = 0f;

            for (int i = 0; i < anchors.Count; i++)
            {
                Vector3 anchor = anchors[i];
                float desiredDistance = distances[i];

                Vector3 dirToAnchor = guess - anchor;
                float currentDist = dirToAnchor.magnitude;
                if (currentDist == 0) continue;

                float error = currentDist - desiredDistance;
                gradient += (error / currentDist) * dirToAnchor;
                totalError += Mathf.Abs(error);
            }

            gradient /= anchors.Count;

            // Apply constraint: Move slightly toward lowest possible Y position
            Vector3 gravityStep = gravityDir.normalized * stepSize;

            // Combined gradient step: respecting constraints + gravity direction
            guess -= stepSize * gradient;
            guess += gravityStep;

            // Evaluate position after gravity step
            float avgError = totalError / anchors.Count;

            // Track best (lowest and accurate) solution found so far
            if (avgError < bestError || (Mathf.Approximately(avgError, bestError) && Vector3.Dot(gravityDir, guess - bestGuess) > 0))
            {
                bestError = avgError;
                bestGuess = guess;
            }

            // Early exit if solution accurate enough
            if (avgError < threshold)
                break;

            // Gradually reduce step size for convergence
            stepSize *= 0.99f;
        }

        return bestGuess;
    }


    private void OnDrawGizmos()
    {
        //Gizmos.color = Color.yellow;
        //for (int i = 0; i < handleSystems.Count; i++)
        //{
        //    Gizmos.DrawWireSphere(getLastPulleyPosition(handleSystems[i]), handleSystems[i].currentTargetPulleyLength);
        //}
    }
}
