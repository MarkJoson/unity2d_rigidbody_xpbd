using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using UnityEngine;

public class EnvironGenerator : MonoBehaviour
{
    public List<PolyShape> polyShapes;
    public GameObject template;
    public int numPolygons = 16;

    public void GeneratePolygon()
    {
        var go = Instantiate(template, transform);
        go.name = $"PolyShape {polyShapes.Count}";
        var poly = go.GetComponent<PolyShape>();
        polyShapes.Add(poly);
        poly.MorphIntoPolygon(Random.Range(3, 8));
        poly.transform.position = new Vector3(Random.Range(-10, 10), Random.Range(-10, 10), 0);
    }

    void Awake()
    {
        for (int i = 0; i < numPolygons; i++)
        {
            GeneratePolygon();
        }
        polyShapes[0].Vertices = new List<Vector2> {
            new Vector2(-0.38f,-3.11f),
            new Vector2(2.66f,-0.30522f),
            new Vector2(0.35004f,2.65f),
            new Vector2(-3.07f,1.52244f),
            new Vector2(-3.99f,-2.61f),
        };

        polyShapes[1].Vertices = new List<Vector2> {
            new Vector2(0.6996f,-1.4672f),
            new Vector2(1.9455f,0.2889f),
            new Vector2(1.4964f,1.38144f),
            new Vector2(-1.099627f,0.8420076f),
            new Vector2(-1.515298f,0.7301468f),
        };
    }

    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }
}
