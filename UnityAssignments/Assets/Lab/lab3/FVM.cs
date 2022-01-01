using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;



public class FVM : MonoBehaviour
{
    float dt = 0.003f;
    float mass = 1;
    float stiffness_0 = 20000.0f;
    float stiffness_1 = 5000.0f;
    float damp = 0.999f;

    float restitution = 1f;                   // for collision

    Vector3 gravity = new Vector3(0, -9.8f, 0);

    int[] Tet;
    int tet_number;         //The number of tetrahedra

    Vector3[] Force;
    Vector3[] V;
    Vector3[] X;
    int number;             //The number of vertices

    Matrix4x4[] inv_Dm;

    //For Laplacian smoothing.
    Vector3[] V_sum;
    int[] V_num;

    SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
        // FILO IO: Read the house model from files.
        // The model is from Jonathan Schewchuk's Stellar lib.
        {
            string fileContent = File.ReadAllText("Assets/Lab/lab3/house2.ele");
            string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

            tet_number = int.Parse(Strings[0]);
            Tet = new int[tet_number * 4];

            for (int tet = 0; tet < tet_number; tet++)
            {
                Tet[tet * 4 + 0] = int.Parse(Strings[tet * 5 + 4]) - 1;
                Tet[tet * 4 + 1] = int.Parse(Strings[tet * 5 + 5]) - 1;
                Tet[tet * 4 + 2] = int.Parse(Strings[tet * 5 + 6]) - 1;
                Tet[tet * 4 + 3] = int.Parse(Strings[tet * 5 + 7]) - 1;
            }
        }
        {
            string fileContent = File.ReadAllText("Assets/Lab/lab3/house2.node");
            string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
            number = int.Parse(Strings[0]);
            X = new Vector3[number];
            for (int i = 0; i < number; i++)
            {
                X[i].x = float.Parse(Strings[i * 5 + 5]) * 0.4f;
                X[i].y = float.Parse(Strings[i * 5 + 6]) * 0.4f;
                X[i].z = float.Parse(Strings[i * 5 + 7]) * 0.4f;
            }
            //Centralize the model.
            Vector3 center = Vector3.zero;
            for (int i = 0; i < number; i++) center += X[i];
            center = center / number;
            for (int i = 0; i < number; i++)
            {
                X[i] -= center;
                float temp = X[i].y;
                X[i].y = X[i].z;
                X[i].z = temp;
            }
        }
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
        Vector3[] vertices = new Vector3[tet_number * 12];
        int vertex_number = 0;
        for (int tet = 0; tet < tet_number; tet++)
        {
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        }

        int[] triangles = new int[tet_number * 12];
        for (int t = 0; t < tet_number * 4; t++)
        {
            triangles[t * 3 + 0] = t * 3 + 0;
            triangles[t * 3 + 1] = t * 3 + 1;
            triangles[t * 3 + 2] = t * 3 + 2;
        }
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();


        V = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

        //TODO: Need to allocate and assign inv_Dm
        inv_Dm = new Matrix4x4[tet_number];
        for (int tet = 0; tet < tet_number; tet++)
            inv_Dm[tet] = Build_Edge_Matrix(tet);
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
        Matrix4x4 ret = Matrix4x4.zero;
        //TODO: Need to build edge matrix here.

        int index_0 = Tet[tet * 4 + 0];
        int index_1 = Tet[tet * 4 + 1];
        int index_2 = Tet[tet * 4 + 2];
        int index_3 = Tet[tet * 4 + 3];

        Vector3 X10 = X[index_1] - X[index_0];
        Vector3 X20 = X[index_2] - X[index_0];
        Vector3 X30 = X[index_3] - X[index_0];

        ret[0, 0] = X10[0];
        ret[1, 0] = X10[1];
        ret[2, 0] = X10[2];

        ret[0, 1] = X20[0];
        ret[1, 1] = X20[1];
        ret[2, 1] = X20[2];

        ret[0, 2] = X30[0];
        ret[1, 2] = X30[1];
        ret[2, 2] = X30[2];

        ret[3, 3] = 1;
        ret = ret.inverse;
        
        return ret;
    }


    void _Update()
    {
        // Jump up.
        if (Input.GetKeyDown(KeyCode.Space))
        {
            for (int i = 0; i < number; i++)
                V[i].y += 0.2f;
        }

        for (int i = 0; i < number; i++)
        {
            //TODO: Add gravity to Force.
            //Force[i] = new Vector3(0, -9.8f, 0);
        }

        for (int tet = 0; tet < 5; tet++)
        {
            //TODO: Deformation Gradient
            Matrix4x4 F = Matrix4x4.zero;
            Matrix4x4 d = Matrix4x4.zero;

            int index_0 = Tet[tet * 4 + 0];
            int index_1 = Tet[tet * 4 + 1];
            int index_2 = Tet[tet * 4 + 2];
            int index_3 = Tet[tet * 4 + 3];

            Vector3 X10 = X[index_1] - X[index_0];
            Vector3 X20 = X[index_2] - X[index_0];
            Vector3 X30 = X[index_3] - X[index_0];

            d[0, 0] = X10[0];
            d[1, 0] = X10[1];
            d[2, 0] = X10[2];
            
            d[0, 1] = X20[0];
            d[1, 1] = X20[1];
            d[2, 1] = X20[2];
           
            d[0, 2] = X30[0];
            d[1, 2] = X30[1];
            d[2, 2] = X30[2];

            d[3, 3] = 1;
            F = d * inv_Dm[tet];
            Debug.Log("F:" + tet);
            Debug.Log(F);

            //TODO: Green Strain
            Matrix4x4 FtF = F.transpose * F;
            Debug.Log("FtF:"+tet);
            Debug.Log(FtF);

            Matrix4x4 G = MatrixMultiple(MatrixSubtraction(FtF, Matrix4x4.identity), 0.5f);
            Debug.Log("G:"+tet);
            Debug.Log(G);
            //TODO: Second PK Stress
            float trace;
            trace = G[0, 0] + G[1, 1] + G[2, 2];
            Debug.Log("trace:" + tet);
            Debug.Log(trace);
            Matrix4x4 dWdG = MatrixAddition(MatrixMultiple(G, 2*stiffness_1),MatrixMultiple(Matrix4x4.identity,stiffness_0*trace));
            Debug.Log("dWdG:" + dWdG);
            Debug.Log(dWdG);
            Matrix4x4 P = F * dWdG;
            Debug.Log("P:" + P);
            Debug.Log(P);

            //TODO: Elastic Force
            float p = -1 / (6 * inv_Dm[tet].determinant);

            Matrix4x4 F123 = MatrixMultiple(P * inv_Dm[tet].transpose, p);
            Debug.Log("F123:" + F123);
            Debug.Log(F123);

            Force[index_1] = new Vector3(F123[0, 0], F123[1, 0], F123[2, 0]);
            Force[index_2] = new Vector3(F123[0, 1], F123[1, 1], F123[2, 1]);
            Force[index_3] = new Vector3(F123[0, 2], F123[1, 2], F123[2, 2]);

            Force[index_0] = -Force[index_1] - Force[index_2] - Force[index_3];

            // test
            V[index_0] += (Force[index_0] + gravity) / mass * dt;
            X[index_0] += V[index_0] * dt;
            V[index_0] *= damp;

            V[index_1] += (Force[index_1]+ gravity) / mass * dt;
            X[index_1] += V[index_1] * dt;
            V[index_1] *= damp;

            V[index_2] += (Force[index_2]+ gravity) / mass * dt;
            X[index_2] += V[index_2] * dt;
            V[index_2] *= damp;

            V[index_3] += (Force[index_3] + gravity) / mass * dt;
            X[index_3] += V[index_3] * dt;
            V[index_3] *= damp;
            Collision_Impulse(new Vector3(0, -3, 0), new Vector3(0, 1, 0));
            Debug.LogWarning("next");
        }

        //for (int i = 0; i < number; i++)
        //{
        //    //TODO: Update X and V here.
        //    V[i] += Force[i]/mass * dt;
        //    X[i] += V[i] * dt;
        //    V[i] *= damp;

        //    //TODO: (Particle) collision with floor.
        //    Collision_Impulse(new Vector3(0, -3, 0), new Vector3(0, 1, 0));
        //}
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

    float PlaneSignedDistanceFunction(Vector3 x, Vector3 P, Vector3 N)
    {
        return Vector3.Dot((x - P), N);
    }

    // Update is called once per frame
    void Update()
    {
        for (int l = 0; l < 10; l++)
            _Update();

        // Dump the vertex array for rendering.
        Vector3[] vertices = new Vector3[tet_number * 12];
        int vertex_number = 0;
        for (int tet = 0; tet < tet_number; tet++)
        {
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        }
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.vertices = vertices;
        mesh.RecalculateNormals();
    }

    Matrix4x4 MatrixSubtraction(Matrix4x4 m1, Matrix4x4 m2)
    {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                m1[i, j] -= m2[i, j];
            }
        }
        m1[3, 3] = 1;
        return m1;
    }

    Matrix4x4 MatrixAddition(Matrix4x4 m1, Matrix4x4 m2)
    {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                m1[i, j] += m2[i, j];
            }
        }
        m1[3, 3] = 1;
        return m1;
    }
    Matrix4x4 MatrixMultiple(Matrix4x4 m, float k)
    {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                m[i, j] *= k;
            }
        }
        m[3, 3] = 1;
        return m;
    }

}
