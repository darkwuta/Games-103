using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RigidBodyDynamics : MonoBehaviour
{
    public float MagOfForce = 10;
    private Rigidbody rigidbody;

    public enum Integration {SemiImplicit, LeapFrog}
    public Integration IntegrationMethod = Integration.SemiImplicit;

    [Header("Physic Properties")]
    public float mass = 1.0f;                   // 8 verticies
    Matrix4x4 I_ref;							// reference inertia

    public Vector3 force;
    public Vector3 velocity;

    public Vector3 torque;
    public Vector3 omega;

  
    // Start is called before the first frame update
    void Start()
    {
        if (this.GetComponent<Rigidbody>())
            rigidbody = this.GetComponent<Rigidbody>();

        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        float m = mass / vertices.Length;     //每个顶点重0.125
        mass = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            mass += m;
            float diag = m * vertices[i].sqrMagnitude;
            I_ref[0, 0] += diag;
            I_ref[1, 1] += diag;
            I_ref[2, 2] += diag;
            I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
            I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
            I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
            I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
            I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
            I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
            I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
            I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
            I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
        }
        I_ref[3, 3] = 1;
    }

    // Update is called once per frame
    void FixedUpdate()
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
        {
            // 上下左右
            if (Input.GetKey(KeyCode.LeftArrow))
                AddForce(new Vector3(-1, 0, 0) * MagOfForce);
            if (Input.GetKey(KeyCode.RightArrow))
                AddForce(new Vector3(1, 0, 0) * MagOfForce);
            if (Input.GetKey(KeyCode.UpArrow))
                AddForce(new Vector3(0, 1, 0) * MagOfForce);
            if (Input.GetKey(KeyCode.DownArrow))
                AddForce(new Vector3(0, -1, 0) * MagOfForce);

            // 旋转 
            if (Input.GetKey(KeyCode.R))
                AddTorque(new Vector3(Mathf.Sin(Time.time), Mathf.Sin(Time.time), Mathf.Sin(Time.time)));

            // 前后
            if (Input.GetKey(KeyCode.Space))
                AddForce(new Vector3(0, 0, 1) * MagOfForce);
            if (Input.GetKey(KeyCode.LeftShift))
                AddForce(new Vector3(0, 0, -1) * MagOfForce);

            Simulate();
        }
            

    }

    void Simulate()
    {
        if(IntegrationMethod == Integration.LeapFrog)
        {
            Vector3 LastVelocity = velocity - force / mass * Time.fixedDeltaTime / 2;//v-0.5 = v0 - dt/2 * f0
            velocity = LastVelocity + force / mass * Time.fixedDeltaTime;//v0.5 = v-0.5 + dt * f0

            Vector3 position = transform.position + velocity * Time.fixedDeltaTime;//x1 = x0 + dt * v0.5
            transform.position = position;
        }
        else if(IntegrationMethod == Integration.SemiImplicit)
        {
            velocity += force / mass * Time.fixedDeltaTime;//v1 = v0 + dt * f0

            Vector3 position = transform.position + velocity * Time.fixedDeltaTime;//x1 = x0 + dt * v1
            transform.position = position;
        }

        // Rotation
        omega += Time.fixedDeltaTime * I_ref.inverse.MultiplyVector(torque);
        Quaternion q = QuaternionAdd(transform.rotation, 
                            new Quaternion(Time.fixedDeltaTime * 1 / 2 * omega.x, Time.fixedDeltaTime * 1 / 2 * omega.y, Time.fixedDeltaTime * 1 / 2 * omega.z, 0) * transform.rotation);
        transform.rotation = q;
        //Drag
        velocity *= 0.999f;
        omega *= 0.98f;
        //力归零
        force = new Vector3(0, 0, 0);
        torque = new Vector3(0, 0, 0);
    }

    void AddForce(Vector3 f)
    {
        force = f;
    }

    void AddTorque(Vector3 t)
    {
        torque = t;
    }

    Quaternion QuaternionAdd(Quaternion q1, Quaternion q2)
    {
        return new Quaternion(q1.x + q2.x, q1.y + q2.y, q1.z + q2.z, q1.w + q2.w);
    }

}
