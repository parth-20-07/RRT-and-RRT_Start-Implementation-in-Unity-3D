using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

[RequireComponent(typeof(BoxCollider))]
public class Obstacle : MonoBehaviour
{
	//MeshRenderer  mesh;
	//BoxCollider  bounding_box;
	void Awake ()
	{
		//bounding_box = GetComponent<BoxCollider>();
		//MeshRenderer[] childrenmeshes = GetComponentsInChildren<MeshRenderer>();

		//int meshrend_vol = int.MaxValue;
		//foreach (var meshrend in childrenmeshes)
		//{
		//	if (meshrend.bounds.size.x * meshrend.bounds.size.y * meshrend.bounds.size.z < meshrend_vol)
		//		mesh = meshrend;
		//}

		//bounding_box.size = mesh.bounds.size;
		gameObject.layer = LayerMask.NameToLayer("Obstacles");
	}

}
