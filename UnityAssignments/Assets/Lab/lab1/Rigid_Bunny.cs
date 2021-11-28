using UnityEngine;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	public bool launched 		= false;
	public float dt 			= 0.015f;
	public Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	public Vector3 w 			= new Vector3(0, 0, 0); // angular velocity
	public bool isGravity		= false;
	private Vector3 gravity		= new Vector3(0.0f, 0.0f, 0.0f);


	float mass;                                 // mass
	float mass_inv;
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision


	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
		mass_inv = 1/mass;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		// P 是平面的位置，N是平面的法相
		// Collision Detection
		Vector3 x = transform.position;
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		Matrix4x4 I = R * I_ref * R.transpose;
		// 遍历顶点
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3 avg_r = new Vector3();
		int collision_num = 0;
		Vector3[] vertices = mesh.vertices;
		for (int i = 0; i < vertices.Length; i++)
        {
			Vector3 Rr_i = R * vertices[i];
			float sdf = PlaneSignedDistanceFunction(x + Rr_i, P, N);
			if (sdf<0)// sdf < 0说明发生了碰撞
            {

				Vector3 r_i = vertices[i];
				
				avg_r += r_i;
				collision_num++;
			}
        }
		if (collision_num == 0)
			return;

		avg_r /= collision_num;
		//Debug.Log(avg_x);

		//  计算新的速度
		Vector3 ri = avg_r;
		Vector3 vi = v + Vector3.Cross(w, R * ri);
		if (Vector3.Dot(vi, N) < 0)
		{
			Vector3 v_Ni = Vector3.Dot(vi, N) * N;
			Vector3 v_Ti = vi - v_Ni;
			Vector3 v_new_i = v_Ni + v_Ti;
			float mu_N = restitution;
			float mu_T = 0.1f;
			float a = Mathf.Max(1 - (mu_T * (1 + mu_N) * v_Ni.magnitude / v_Ti.magnitude), 0);
			v_Ni = -mu_N * v_Ni;
			v_Ti = a * v_Ti;
			v_new_i = v_Ni + v_Ti;

			// 计算矩阵K
			Matrix4x4 crossRri = Get_Cross_Matrix(R * ri);
			Matrix4x4 Identity = Matrix4x4.identity;
			Identity[0, 0] = mass_inv;
			Identity[1, 1] = mass_inv;
			Identity[2, 2] = mass_inv;
			Identity[3, 3] = mass_inv;
			Matrix4x4 K = MatrixSubtraction(Identity, crossRri * I.inverse * crossRri);

			// 计算j
			Vector3 J = K.inverse * (v_new_i - vi);

			// 每一个点都有重力，但是下面的点受力后速度会反弹
			v += J * mass_inv;
			Vector3 dw = I.inverse * crossRri * J;

			//角速度会发生突变
			w += dw;
			Debug.Log(w);
		}

		//Vector3 ri = avg_x;// 这里很重要，要用局部坐标
		//Vector3 vi = v + Vector3.Cross(w, R * ri);


	}

	// Update is called once per frame
	void FixedUpdate () 
	{
		
		//Game Control
		if (Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (4, 0, 0);
			w = new Vector3(0f, 0f, 0f);
			launched =true;
		}

		
		if (launched)
		{
			// Part I: Update velocities
			// LeapForg
			if (isGravity)
				gravity = new Vector3(0.0f, -9.8f, 0.0f);
			else
				gravity = new Vector3(0.0f, 0.0f, 0.0f);
			v = v + gravity * dt/2;

			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

			if (v.magnitude <= 0.01)
				v = new Vector3();
			if (w.magnitude <= 0.01)
				w = new Vector3();

			// Part III: Update position & orientation
			//Update linear status
			Vector3 x = transform.position;
			x += v * dt;
			v = v + gravity * dt / 2;
			//Update angular status
			Quaternion q = transform.rotation;
			Quaternion dq = new Quaternion(dt * 1 / 2 * w.x, dt * 1 / 2 * w.y, dt * 1 / 2 * w.z, 0) * q;
			q = QuaternionAdd(q, dq);
			//q.Normalize();

			// Part IV: Assign to the object
			transform.position = x;
			transform.rotation = q;
			w *= angular_decay;
			v *= linear_decay;
			
		}
	}
	Quaternion QuaternionAdd(Quaternion q1, Quaternion q2)
	{
		return new Quaternion(q1.x + q2.x, q1.y + q2.y, q1.z + q2.z, q1.w + q2.w);
	}

	float PlaneSignedDistanceFunction(Vector3 x, Vector3 P, Vector3 N)
    {
		return Vector3.Dot((x - P), N);
    }
	Matrix4x4 MatrixMultiplication(float k, Matrix4x4 m)
    {
		m[0, 0] *= k;
		m[0, 1] *= k;
		m[0, 2] *= k;

		m[1, 0] *= k;
		m[1, 1] *= k;
		m[1, 2] *= k;

		m[2, 0] *= k;
		m[2, 1] *= k;
		m[2, 2] *= k;

		m[3, 3] = 1;

		return m;
    }

	Matrix4x4 MatrixSubtraction(Matrix4x4 m1, Matrix4x4 m2)
	{
		//m1[0, 0] -= m2[0, 0];
		//m1[0, 1] -= m2[0, 1];
		//m1[0, 2] -= m2[0, 2];
		 		 		
		//m1[1, 0] -= m2[1, 0];
		//m1[1, 1] -= m2[1, 1];
		//m1[1, 2] -= m2[1, 2];
		 		 		
		//m1[2, 0] -= m2[2, 0];
		//m1[2, 1] -= m2[2, 1];
		//m1[2, 2] -= m2[2, 2];

		for (int x = 0; x < 4; x++) 
        {
			for (int y = 0; y < 4; y++) 
            {
				m1[x, y] -= m2[x, y];
			}
        }

		return m1;
	}
}