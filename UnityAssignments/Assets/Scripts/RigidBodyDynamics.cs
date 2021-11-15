using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RigidBodyDynamics : MonoBehaviour
{
    public float MagOfForce = 10;
    private Rigidbody rigidbody;

    [SerializeField]
    private Vector3 velocity;
    [SerializeField]
    private Quaternion angularVelocity;

  
    // Start is called before the first frame update
    void Start()
    {
        if (this.GetComponent<Rigidbody>())
            rigidbody = this.GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        if (this.GetComponent<Rigidbody>())
        {
            // 上下左右
            if (Input.GetKey(KeyCode.LeftArrow))
                rigidbody.AddForce(new Vector3(-1, 0, 0) * MagOfForce);
            if (Input.GetKey(KeyCode.RightArrow))
                rigidbody.AddForce(new Vector3(1, 0, 0) * MagOfForce);
            if (Input.GetKey(KeyCode.UpArrow))
                rigidbody.AddForce(new Vector3(0, 1, 0) * MagOfForce);
            if (Input.GetKey(KeyCode.DownArrow))
                rigidbody.AddForce(new Vector3(0, -1, 0) * MagOfForce);

            // 旋转 
            if (Input.GetKey(KeyCode.R))
                rigidbody.AddTorque(new Vector3(Mathf.Sin(Time.time), Mathf.Sin(Time.time), Mathf.Sin(Time.time)));

            // 前后
            if (Input.GetKey(KeyCode.Space))
                rigidbody.AddForce(new Vector3(0, 0, 1) * MagOfForce);
            if (Input.GetKey(KeyCode.LeftShift))
                rigidbody.AddForce(new Vector3(0, 0, -1) * MagOfForce);
        }
            
        else
            Simulation();

    }

    void Simulation()
    {

    }
}
