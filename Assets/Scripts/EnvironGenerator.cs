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
    }

    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }
}
