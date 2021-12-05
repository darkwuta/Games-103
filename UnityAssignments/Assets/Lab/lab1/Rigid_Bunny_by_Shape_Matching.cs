using System.Collections;
using System.Collections.Generic;
using UnityEngine;
struct rowVector3
{
	public float x;
	public float y;
	public float z;
	public rowVector3(float x, float y,float z)
    {
		this.x = x;
		this.y = y;
		this.z = z;
    }
	public rowVector3(Vector3 v)
    {
		this.x = v.x;
		this.y = v.y;
		this.z = v.z;
    }
	public static Matrix4x4 operator *(Vector3 a, rowVector3 b)
    {
		Matrix4x4 M = Matrix4x4.zero;
		M[0, 0] = a.x * b.x;
		M[0, 1] = a.x * b.y;
		M[0, 2] = a.x * b.z;

		M[1, 0] = a.y * b.x;
		M[1, 1] = a.y * b.y;
		M[1, 2] = a.y * b.z;

		M[2, 0] = a.z * b.x;
		M[2, 1] = a.z * b.y;
		M[2, 2] = a.z * b.z;

		M[3, 3] = 1;
		return M;
    }


}
public class Rigid_Bunny_by_Shape_Matching : MonoBehaviour
{


	public bool isGravity = false;
	private Vector3 gravity = new Vector3(0.0f, 0.0f, 0.0f);
	public bool launched = false;

	public ComputeShader computeShader;
	public RenderTexture renderTexture;
	Vector3[] X;
	Vector3[] Q;
	Vector3[] V;
	Matrix4x4 QQt = Matrix4x4.zero;

	private Vector3 omega = new Vector3();


	float linear_decay = 0.999f;                // for velocity decay
	float angular_decay = 0.98f;
	float restitution = 0.8f;                   // for collision

	// Start is called before the first frame update
	void Start()
    {
		//renderTexture = new RenderTexture(256, 256, 24);
		//renderTexture.enableRandomWrite = true;
		//renderTexture.Create();

		//computeShader.SetTexture(0, "Result", renderTexture);
		//computeShader.Dispatch(0, renderTexture.width / 8, renderTexture.height / 8, 1);

    	Mesh mesh = GetComponent<MeshFilter>().mesh;
        V = new Vector3[mesh.vertices.Length];
        X = mesh.vertices;
        Q = mesh.vertices;

        //Centerizing Q.
        Vector3 c=Vector3.zero;
        for(int i=0; i<Q.Length; i++)
        	c+=Q[i];
        c/=Q.Length;
        for(int i=0; i<Q.Length; i++)
        	Q[i]-=c;

        //Get QQ^t ready.
		for(int i=0; i<Q.Length; i++)
		{
			QQt[0, 0]+=Q[i][0]*Q[i][0];
			QQt[0, 1]+=Q[i][0]*Q[i][1];
			QQt[0, 2]+=Q[i][0]*Q[i][2];
			QQt[1, 0]+=Q[i][1]*Q[i][0];
			QQt[1, 1]+=Q[i][1]*Q[i][1];
			QQt[1, 2]+=Q[i][1]*Q[i][2];
			QQt[2, 0]+=Q[i][2]*Q[i][0];
			QQt[2, 1]+=Q[i][2]*Q[i][1];
			QQt[2, 2]+=Q[i][2]*Q[i][2];
		}
		QQt[3, 3]=1;

		for(int i=0; i<X.Length; i++)
			V[i][0]=4.0f;

		Update_Mesh(transform.position, Matrix4x4.Rotate(transform.rotation), 0);
		transform.position=Vector3.zero;
		transform.rotation=Quaternion.identity;
   	}

   	// Polar Decomposition that returns the rotation from F.
   	Matrix4x4 Get_Rotation(Matrix4x4 F)
	{
		Matrix4x4 C = Matrix4x4.zero;
	    for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        C[ii,jj]+=F[kk,ii]*F[kk,jj];
	   
	   	Matrix4x4 C2 = Matrix4x4.zero;
		for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        C2[ii,jj]+=C[ii,kk]*C[jj,kk];
	    
	    float det    =  F[0,0]*F[1,1]*F[2,2]+
	                    F[0,1]*F[1,2]*F[2,0]+
	                    F[1,0]*F[2,1]*F[0,2]-
	                    F[0,2]*F[1,1]*F[2,0]-
	                    F[0,1]*F[1,0]*F[2,2]-
	                    F[0,0]*F[1,2]*F[2,1];
	    
	    float I_c    =   C[0,0]+C[1,1]+C[2,2];
	    float I_c2   =   I_c*I_c;
	    float II_c   =   0.5f*(I_c2-C2[0,0]-C2[1,1]-C2[2,2]);
	    float III_c  =   det*det;
	    float k      =   I_c2-3*II_c;
	    
	    Matrix4x4 inv_U = Matrix4x4.zero;
	    if(k<1e-10f)
	    {
	        float inv_lambda=1/Mathf.Sqrt(I_c/3);
	        inv_U[0,0]=inv_lambda;
	        inv_U[1,1]=inv_lambda;
	        inv_U[2,2]=inv_lambda;
	    }
	    else
	    {
	        float l = I_c*(I_c*I_c-4.5f*II_c)+13.5f*III_c;
	        float k_root = Mathf.Sqrt(k);
	        float value=l/(k*k_root);
	        if(value<-1.0f) value=-1.0f;
	        if(value> 1.0f) value= 1.0f;
	        float phi = Mathf.Acos(value);
	        float lambda2=(I_c+2*k_root*Mathf.Cos(phi/3))/3.0f;
	        float lambda=Mathf.Sqrt(lambda2);
	        
	        float III_u = Mathf.Sqrt(III_c);
	        if(det<0)   III_u=-III_u;
	        float I_u = lambda + Mathf.Sqrt(-lambda2 + I_c + 2*III_u/lambda);
	        float II_u=(I_u*I_u-I_c)*0.5f;
	        
	        
	        float inv_rate, factor;
	        inv_rate=1/(I_u*II_u-III_u);
	        factor=I_u*III_u*inv_rate;
	        
	       	Matrix4x4 U = Matrix4x4.zero;
			U[0,0]=factor;
	        U[1,1]=factor;
	        U[2,2]=factor;
	        
	        factor=(I_u*I_u-II_u)*inv_rate;
	        for(int i=0; i<3; i++)
	        for(int j=0; j<3; j++)
	            U[i,j]+=factor*C[i,j]-inv_rate*C2[i,j];
	        
	        inv_rate=1/III_u;
	        factor=II_u*inv_rate;
	        inv_U[0,0]=factor;
	        inv_U[1,1]=factor;
	        inv_U[2,2]=factor;
	        
	        factor=-I_u*inv_rate;
	        for(int i=0; i<3; i++)
	        for(int j=0; j<3; j++)
	            inv_U[i,j]+=factor*U[i,j]+inv_rate*C[i,j];
	    }
	    
	    Matrix4x4 R=Matrix4x4.zero;
	    for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        R[ii,jj]+=F[ii,kk]*inv_U[kk,jj];
	    R[3,3]=1;
	    return R;
	}

	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A[0, 0] = 0;
		A[0, 1] = -a[2];
		A[0, 2] = a[1];
		A[1, 0] = a[2];
		A[1, 1] = 0;
		A[1, 2] = -a[0];
		A[2, 0] = -a[1];
		A[2, 1] = a[0];
		A[2, 2] = 0;
		A[3, 3] = 1;
		return A;
	}
	// Update the mesh vertices according to translation c and rotation R.
	// It also updates the velocity.
	void Update_Mesh(Vector3 c, Matrix4x4 R, float inv_dt)
   	{
   		for(int i=0; i<Q.Length; i++)
		{
			Vector3 x=(Vector3)(R * Q[i])+c;// 顶点的位置

			V[i]+=(x-X[i])*inv_dt;
			X[i]=x;
		}	
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices=X;
   	}

	float PlaneSignedDistanceFunction(Vector3 x, Vector3 P, Vector3 N)
	{
		return Vector3.Dot((x - P), N);
	}

	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		for (int i = 0; i < V.Length; i++)
		{
			float sdf = PlaneSignedDistanceFunction(X[i], P, N);
			if (sdf < 0)
            {
				if (Vector3.Dot(V[i], N) < 0)
                {
					Vector3 v_Ni = Vector3.Dot(V[i], N) * N;
					Vector3 v_Ti = V[i] - v_Ni;
					Vector3 v_new_i = v_Ni + v_Ti;
					float mu_N = restitution;
					float mu_T = 0.1f;
					float a = Mathf.Max(1 - (mu_T * (1 + mu_N) * v_Ni.magnitude / v_Ti.magnitude), 0);
					v_Ni = -mu_N * v_Ni;
					v_Ti = a * v_Ti;
					v_new_i = v_Ni + v_Ti;

					V[i] = v_new_i;

					X[i] = X[i] - sdf * N;
				}
			}
		}
	}
	void Collision(float inv_dt)
	{
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));
	}

    // Update is called once per frame
    void Update()
    {

		//Game Control
		if (Input.GetKey("r"))
		{
			transform.position = new Vector3(0, 0.6f, 0);
			restitution = 0.5f;
			launched = false;
		}
		if (Input.GetKey("l"))
		{
			omega = new Vector3(0.5f, 1.5f, 0.0f);
			for (int i = 0; i < X.Length; i++)
            {
				V[i][0] = 4.0f;
				V[i][1] = 3.8f;
			}
				
			launched = true;
		}


		if (launched)
		{
			if (isGravity)
				gravity = new Vector3(0.0f, -9.8f, 0.0f);//没有重力时旋转很正常，但是有了重力头部就会一直朝下
			else
				gravity = new Vector3(0.0f, 0.0f, 0.0f);

			float dt = 0.015f;
			Vector3 c = new Vector3();
			Matrix4x4 A = Matrix4x4.zero;
			Matrix4x4 R = Matrix4x4.zero;
			//Step 1: run a simple particle system.
			for (int i = 0; i < V.Length; i++)
			{
				// simi-integration
				V[i] += gravity * dt;
				X[i] += V[i] * dt;
				V[i] *= linear_decay;
			}
			//Step 2: Perform simple particle collision.
			Collision(1 / dt);

			// Step 3: Use shape matching to get new translation c and 
			// new rotation R. Update the mesh by c and R.
			//Shape Matching (translation)
			for (int i = 0; i < V.Length; i++)
			{
				c += X[i];
			}
			c /= V.Length;

			Matrix4x4 a_left = Matrix4x4.zero;
			Matrix4x4 a_right = QQt.inverse;
			for (int i = 0; i < V.Length; i++)
			{
				//rowVector3 ri_transpose = new rowVector3(Q[i]);
				//a_left = MatrixAddition(a_left,(X[i] - c) * ri_transpose);
				a_left[0, 0] += (X[i] - c).x * Q[i].x;
				a_left[0, 1] += (X[i] - c).x * Q[i].y;
				a_left[0, 2] += (X[i] - c).x * Q[i].z;
							 
				a_left[1, 0] += (X[i] - c).y * Q[i].x;
				a_left[1, 1] += (X[i] - c).y * Q[i].y;
				a_left[1, 2] += (X[i] - c).y * Q[i].z;
							 
				a_left[2, 0] += (X[i] - c).z * Q[i].x;
				a_left[2, 1] += (X[i] - c).z * Q[i].y;
				a_left[2, 2] += (X[i] - c).z * Q[i].z;

				a_left[3, 3] = 1;

			}
			A = a_left * a_right;

			//Shape Matching (rotation)
			R = Get_Rotation(A);

			Update_Mesh(c, R, 1 / dt);
		}
		
    }

	Matrix4x4 MatrixAddition(Matrix4x4 A, Matrix4x4 B)
    {
		for (int x = 0; x < 4; x++)
		{
			for (int y = 0; y < 4; y++)
			{
				A[x, y] += B[x, y];
			}
		}
		return A;
	}
}
