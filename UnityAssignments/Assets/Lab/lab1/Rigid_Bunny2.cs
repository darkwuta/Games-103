using UnityEngine;
using System.Collections;

public class Rigid_Bunny2 : MonoBehaviour
{
    public enum Integrate
    {
        leapforg,
        semi_implicit,
    }
    public Integrate Method = Integrate.leapforg;

    bool launched = false;
    float dt = 0.015f;
    Vector3 v = new Vector3(0, 0, 0);   // velocity
    Vector3 w = new Vector3(0, 0, 0);   // angular velocity

    float mass;                                  // mass
    float M_inv;                                 // 1/mass
    Matrix4x4 I_ref;                            // reference inertia

    float linear_decay = 0.999f;                // for velocity decay
    float angular_decay = 0.98f;
    float restitution = 0.5f;                   // for collision


    // Use this for initialization
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        float m = 1;
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
        M_inv = 1 / mass;
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

    Matrix4x4 MatrixSubtraction(Matrix4x4 A, Matrix4x4 B)
    {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
            {
                A[i, j] -= B[i, j];
            }
        return A;
    }

    // In this function, update v and w by the impulse due to the collision with
    //a plane <P, N>
    void Collision_Impulse(Vector3 P, Vector3 N)
    {

    }

    #region Native Collision Impulse For Debug

    void Native_Collision_Impulse1(Vector3 P, Vector3 N)
    {
        Vector3 X = transform.position;
        float sdf = Vector3.Dot(X - P, N);
        if (sdf < 0f)
        {
            X += N * Mathf.Abs(sdf);
        }
        else
        {
            return;
        }
        //transform.position = X;
        sdf = Vector3.Dot(v, N);
        if (sdf < 0f)
        {
            float u = 0.01f;
            float a = 0.01f;
            Vector3 Vn = Vector3.Project(v, N);
            Vector3 Vt = v - Vn;
            Vn *= -u;
            Vt *= a;
            v = Vn + Vt;
        }
        else
        {
            return;
        }
    }

    void Native_Collision_Impulse(Vector3 P, Vector3 N)
    {
        Vector3 X = transform.position;
        float sdf = Vector3.Dot(X - P, N);
        if (sdf < 0f)//position 
        {
            sdf = Vector3.Dot(v, N);
            if (sdf < 0f)//velocity
            {
                float u = restitution;
                float a = 0.5f;
                Vector3 Vn = Vector3.Project(v, N);
                Vector3 Vt = v - Vn;
                //a = Mathf.Max(1-u*(1+u)*Vn.magnitude/Vt.magnitude,0f);
                Vn *= -u;
                Vt *= a;
                v = Vn + Vt;
            }
        }
        else
        {
            return;
        }
    }

    void RigidBody_Collision_Impulse(Vector3 P, Vector3 N)
    {
        //for Compute the impulse j  
        Vector3 xi;
        Vector3 ri;
        Vector3 xi_averge = new Vector3();//average
        Vector3 ri_averge = new Vector3();//average
        Vector3 vi;
        Vector3 vi_new;

        Matrix4x4 K;
        Matrix4x4 Identity = Matrix4x4.identity;
        Identity[0, 0] = M_inv;
        Identity[1, 1] = M_inv;
        Identity[2, 2] = M_inv;
        Identity[3, 3] = M_inv;
        Vector3 j;
        //M_inv is global
        //angular:
        Quaternion q = transform.rotation;
        Matrix4x4 R = Matrix4x4.Rotate(q);
        Matrix4x4 Inertia = R * I_ref * R.transpose;//Inertia

        //TODO : interate vertex to get average ri:
        //...
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        Vector3 x = transform.position;
        float total = 0f;
        float sdf;
        bool flag = false;
        for (int i = 0; i < vertices.Length; ++i)
        {
            ri = vertices[i];
            Vector3 Rri1 = R * ri;
            xi = x + Rri1;
            sdf = Vector3.Dot(xi - P, N);
            if (sdf < 0f)
            {
                vi = v + Vector3.Cross(w, Rri1);
                sdf = Vector3.Dot(vi, N);
                if (sdf < 0f)
                {
                    ri_averge += ri;
                    xi_averge += xi;
                    ++total;
                    flag = true;
                }
            }
        }


        if(!flag)
        {
            return;
        }

        ri_averge /= total;
        xi_averge /= total;
        print("total:" + total);
        print("xi_averge:" + xi_averge); 
        print("ri_averge:" + ri_averge);
        //ri = ri_averge;
        xi = xi_averge;
        //ri = xi - x;
        ri = ri_averge;
        vi = v + Vector3.Cross(w, R * ri);
        sdf = Vector3.Dot(xi - P, N);
        if(sdf < 0)
        {
            sdf = Vector3.Dot(vi, N);
            if (sdf < 0)
            {

                float u = restitution;

                float a;
                Vector3 Vn;
                Vn = Vector3.Dot(vi, N) * N;
                Vector3 Vt = vi - Vn;
                a = Mathf.Max(1 - u * (1 + u) * Vn.magnitude / Vt.magnitude, 0f);
                Vn *= -u;
                Vt *= a;
                vi_new = Vn + Vt;
                //TODO : Compute the impulse j :
                //...
                Vector3 Rri = R * ri;
                Matrix4x4 crossRri = Get_Cross_Matrix(Rri);
                K = MatrixSubtraction(Identity, crossRri * Inertia.inverse * crossRri);
                j = K.inverse * (vi_new - vi);

                //TODO : add j to v and w
                //...
                v += j * M_inv;
                Vector3 wa = (Vector3.Cross(Rri, j));
                Vector3 w_xyz = Inertia.inverse * Vector3.Cross(Rri, j);
                w += w_xyz;
               
                Debug.Log("When collision v:" + v);

            }
        }
}

    void RigidBody_Collision_Impulse1(Vector3 P, Vector3 N)
    {
        //for Compute the impulse j  
        bool flag = false;
        Vector3 ri;
        Vector3 vi;
        Vector3 vi_new;

        Matrix4x4 K;
        Matrix4x4 Identity = Matrix4x4.identity;
        Identity[0, 0] = M_inv;
        Identity[1, 1] = M_inv;
        Identity[2, 2] = M_inv;
        Identity[3, 3] = M_inv;
        Vector3 j;
        //M_inv is global
        //angular:
        Quaternion q = transform.rotation;
        Matrix4x4 R = Matrix4x4.Rotate(q);
        Matrix4x4 Inertia = R * I_ref * R.transpose;//Inertia

        //TODO : interate vertex to get average ri:
        //...
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        Vector3 x = transform.position;
        float total = 0f;
        float sdf;
        for (int i = 0; i < vertices.Length; ++i)
        {
            ri = vertices[i];
            Vector3 Rri1 = R * ri;
            sdf = Vector3.Dot(x + ri - P, N);
            if (sdf < 0f)
            {
                vi = v + Vector3.Cross(w, Rri1);
                sdf = Vector3.Dot(vi, N);
                if (sdf < 0f)
                {
                    flag = true;
                    float u = restitution;
                    float u1 = 0.1f;
                    float a;
                    Vector3 Vn = Vector3.Project(vi, N);
                    Vector3 Vt = vi - Vn;
                    a = Mathf.Max(1 - u1 * (1 + u) * Vn.magnitude / Vt.magnitude, 0f);
                    Vn *= -u;
                    Vt *= a;
                    vi_new = Vn + Vt;

                    //TODO : Compute the impulse j :
                    //...
                    Vector3 Rri = R * ri;
                    Matrix4x4 crossRri = Get_Cross_Matrix(Rri);
                    K = MatrixSubtraction(Identity, crossRri * Inertia.inverse * crossRri);
                    j = K.inverse * (vi_new - vi);
                    //TODO : add j to v and w
                    //...
                    v += j * M_inv;
                    Vector3 w_xyz = Inertia.inverse * Vector3.Cross(Rri, j);
                    w += w_xyz;
                    ++total;
                }
            }
        }
        if (total > 0 && flag)
        {
            v /= total;
            w /= total;
        }

    }

    #endregion

    // Update is called once per frame
    void Update()
    {
        //Game Control
        if (Input.GetKey("r"))
        {
            transform.position = new Vector3(0, 0.6f, 0);
            restitution = 0.5f;
            launched = false;
            print("Pause");
        }
        if (Input.GetKey("l"))
        {
            v = new Vector3(5f, 2f, 0f);
            w = new Vector3(5f, 5f, 5f);//initializing angular velocity
            launched = true;
            print("Start");
        }


        if (launched)
        {
            // Part I: Update velocities
            ///Gravity Update:
            float g = 0.98f * 10f;//tmp ->9.8f
            Vector3 G = new Vector3(0f, -mass * g, 0f);
            if (Method == Integrate.leapforg)
            {
                v.y -= g * dt * 0.5f;
            }
            else// Integrate.semi_implicit
            {
                v.y -= g * dt;
            }
            ///Damp effect:
            v *= linear_decay;
            w *= angular_decay;

            print("Before collision v:" + v);
            // Part II: Collision Impulse
            //Native_Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));//ground
            //Native_Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));//wall
            RigidBody_Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));//ground
            RigidBody_Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));//wall

            Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));//ground
            Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));//wall

            // Part III: Update position & orientation
            //Update linear status
            Vector3 x = transform.position;
            //x.y += v.y * dt;

            x += v * dt;
            if (Method == Integrate.leapforg)
            {
                v.y -= g * dt * 0.5f;
            }

            print("After collision v:" + v);

            //Update angular status
            Quaternion q = transform.rotation;
            //print("qQ" + q);
            //print("rotation1" + transform.rotation);

            //Matrix4x4 R = Matrix4x4.Rotate(q);
            //Matrix4x4 Inertia = R * I_ref * R.transpose;

            float real_part = 0;
            float complex_i = dt * 0.5f * w.x;
            float complex_j = dt * 0.5f * w.y;
            float complex_k = dt * 0.5f * w.z;
            Quaternion tmp1 = new Quaternion();
            //tmp1.x = real_part;
            //tmp1.y = complex_i;
            //tmp1.z = complex_j;
            //tmp1.w = complex_k;
            tmp1.Set(real_part, complex_i, complex_j, complex_k);
            //print("tmp1Q" + tmp1);
            Quaternion tmp2 = new Quaternion();
            tmp2 = q * tmp1;
            //print("tmp2Q" + tmp2);
            q.w += tmp2.w;
            q.x += tmp2.x;
            q.y += tmp2.y;
            q.z += tmp2.z;
            //print("q2Q" + q);
            // Part IV: Assign to the object
            transform.position = x;
            transform.rotation = q;
            //print("rotation2" + transform.rotation);
        }
    }
}
