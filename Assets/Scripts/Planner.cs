using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class Planner : MonoBehaviour
{
	#region Variables
	[Header("Planning Parameters")]
	[SerializeField] private GameObject Car_Object;
	[SerializeField] private GameObject End_Point;

	[Header("Sampling Parameters")]
	[SerializeField] [Range(1, 20)] private float Sampling_Distance_In_m;
	[SerializeField] [Range(1, 40)] private float Neighbor_Updation_Search_Radius_In_m;
	[SerializeField] [Range(1, 5)] private float Acceptable_Error_Distance_In_m;
	[SerializeField] [Range(100, 10000)] private int Max_Iterations;

	[Header("Motion Planner Parameters")]
	private bool Path_Not_Retraced = true;
	private bool Path_Found = false;
	private int Mapping_Layer_Number, Obstacle_Layer_Mask;
	private struct Struct_Node
	{
		public int End_Goal_To_Current_Node_Travel_Cost;//cost = dX^2 + dY^2
		public GameObject Node_Object;//GameObject to hold current object
		public GameObject Parent_Node_Object;//GameObject to hold parent object
		public int Parent_Node_To_Current_Node_Travel_Cost;//cost = dX^2 + dY^2
		public int Node_Index_In_List; //Index of the Node in the Node_List
		public int Parent_Node_Index_In_List; //Index of the Parent Node in the Node_List
		public int Start_To_Current_Node_Travel_Cost;//cost = dX^2 + dY^2
		public LineRenderer Line;
	};

	readonly List<Struct_Node> Mapped_Nodes_Graph = new();//List to hold all the nodes

	[Header("Visualization Parameters")]
	[SerializeField] LineRenderer Line_Renderer;
	[SerializeField] GameObject Search_Marker;
	[SerializeField][Range(0.0f,1.0f)] float Movement_Speed_In_m;
	private Color Complete_Search_Color = Color.red;
	private readonly List<Vector3> Generated_Path = new();
	private int tracing_index = 0;

	#endregion //Variables

	#region MainMethods
	void Start ()
	{
		if (this.Sampling_Distance_In_m == default)
		{
			Debug.LogError("Sampling Distance Not Specified");
			return;
		}
		if (this.Neighbor_Updation_Search_Radius_In_m == default)
		{
			Debug.LogError("Neighbor Search Radius Not Specified");
			return;
		}
		if (this.Neighbor_Updation_Search_Radius_In_m < this.Sampling_Distance_In_m)
		{
			Debug.LogError("Neighbor Search Radius Less Than Sampling Distance");
			return;
		}
		if (this.Acceptable_Error_Distance_In_m == default)
		{
			Debug.LogError("Acceptable Error Not Specified");
			return;
		}
		if (this.Max_Iterations == default)
		{
			Debug.LogError("Max Iterations Limit Not Specified");
			return;
		}

		Mapping_Layer_Number = LayerMask.NameToLayer("Mapping");
		Obstacle_Layer_Mask = LayerMask.GetMask("Obstacles");

		Acceptable_Error_Distance_In_m *= Acceptable_Error_Distance_In_m;
		Neighbor_Updation_Search_Radius_In_m *= Neighbor_Updation_Search_Radius_In_m;

		//Place the Start Node in the List
		Struct_Node node0 = new();
		node0.Node_Object = Instantiate(Search_Marker);
		//node0.Node_Object.transform.localScale = Node_Marker_Size;
		//node0.Node_Object.GetComponent<Renderer>().material.SetColor("_Color", Active_Search_Color);
		node0.Node_Object.transform.SetParent(this.transform);
		node0.Node_Object.layer = this.Mapping_Layer_Number;

		node0.Node_Object.transform.position = Car_Object.transform.position;
		node0.End_Goal_To_Current_Node_Travel_Cost = (int)((node0.Node_Object.transform.position - End_Point.transform.position).sqrMagnitude);
		node0.Parent_Node_Object = default;
		node0.Parent_Node_Index_In_List = default;
		node0.Parent_Node_To_Current_Node_Travel_Cost = 0;
		node0.Start_To_Current_Node_Travel_Cost = 0;
		this.Mapped_Nodes_Graph.Add(node0);
		Debug.Log("Starting Planning");
	}

	void Update ()
	{
		//Implement RRT Algorithm to Scan for Route to the end goal
		Struct_Node open_node = Mapped_Nodes_Graph.Last();
		//print(open_node.Heuristic_Cost);
		//if ((Counter < Max_Iterations) && (open_node.End_Goal_To_Current_Node_Travel_Cost > Acceptable_Error_Distance_In_m) && Path_Not_Found)
		if ((Max_Iterations > 0) && Path_Not_Retraced)
		{
			Max_Iterations--;
			Find_New_Neighbor(open_node);
		}
		//else if ((open_node.End_Goal_To_Current_Node_Travel_Cost < Acceptable_Error_Distance_In_m) && (Path_Not_Found))
		else if (Path_Not_Retraced)
		{
			Path_Not_Retraced = false;
			if (Retrace_Path())
			{
				Plot_Path();
				Path_Found = true;
				Debug.Log("Search Successful!");
			}
			else
				Debug.LogWarning("Search Failed");
		}
		else if (Path_Found)
		{
			Car_Object.transform.position = Vector3.MoveTowards(Car_Object.transform.position, Generated_Path[tracing_index], Movement_Speed_In_m);
			if (Mathf.Abs((Car_Object.transform.position - Generated_Path[tracing_index]).magnitude) < 0.5f)
			{
				tracing_index++;
				if (tracing_index == Generated_Path.Count())
					Path_Found = false;
			}
		}
	}
	#endregion //MainMethods

	#region CustomMethods
	void Find_New_Neighbor (Struct_Node open_node)
	{
		//Cast a ray in random direction to see if ostacle is detected
		int random_angle = Random.Range(0, 360);
		Vector3 ray_direction = open_node.Node_Object.transform.TransformDirection(new(this.Sampling_Distance_In_m * Mathf.Cos(random_angle), 0, this.Sampling_Distance_In_m * Mathf.Sin(random_angle)));
		Ray ray = new(open_node.Node_Object.transform.position, ray_direction);
		Debug.DrawRay(open_node.Node_Object.transform.position, ray_direction);
float Node_Placement_Distance = Sampling_Distance_In_m;
		if (Physics.Raycast(ray, out RaycastHit hit, this.Sampling_Distance_In_m, Obstacle_Layer_Mask))
			Node_Placement_Distance = hit.distance-0.1f;
			//If no obstacle is detected, save the location as a possible path
			//private struct Struct_Node
			//{
			//	public int End_Goal_To_Current_Node_Travel_Cost;//cost = dX^2 + dY^2
			//	public GameObject Node_Object;//GameObject to hold current object
			//	public GameObject Parent_Node_Object;//GameObject to hold parent object
			//	public int Parent_Node_To_Current_Node_Travel_Cost;//cost = dX^2 + dY^2
			//	public int Node_Index_In_List; //Index of the Node in the Node_List
			//	public int Parent_Node_Index_In_List; //Index of the Parent Node in the Node_List
			//	public int Start_To_Current_Node_Travel_Cost;//cost = dX^2 + dY^2
			//};

			//Generate the GameObject to Visualize the Node and set it on Mapping Layer
			Struct_Node new_found_node = new();
		//new_found_node.Node_Object = GameObject.CreatePrimitive(PrimitiveType.Sphere);
		new_found_node.Node_Object = Instantiate(Search_Marker);
		//new_found_node.Node_Object.transform.localScale = Node_Marker_Size;
			//new_found_node.Node_Object.GetComponent<Renderer>().material.SetColor("_Color", Active_Search_Color);
			new_found_node.Node_Object.transform.SetParent(this.transform);
			new_found_node.Node_Object.layer = this.Mapping_Layer_Number;

			new_found_node.Node_Index_In_List = Mapped_Nodes_Graph.Count();//Set the element as last item in the list

			//Update the Location of Node and Add the End Goal Travel Cost
			new_found_node.Node_Object.transform.position = open_node.Node_Object.transform.position + new Vector3(Node_Placement_Distance * Mathf.Cos(random_angle), 0, Node_Placement_Distance * Mathf.Sin(random_angle));
			new_found_node.End_Goal_To_Current_Node_Travel_Cost = (int)((new_found_node.Node_Object.transform.position - End_Point.transform.position).sqrMagnitude);

		//RRT Algorithm
		//Look for the node which is nearest to current node to set as parent
		int Parent_Node_To_Current_Node_Travel_Cost = int.MaxValue;
		for (int index = 0; index < Mapped_Nodes_Graph.Count(); index++)
		{
			Struct_Node object_node = Mapped_Nodes_Graph[index];
			int Temporary_Parent_Node_To_Current_Node_Travel_Cost = (int)((object_node.Node_Object.transform.position - new_found_node.Node_Object.transform.position).sqrMagnitude);
			if (Temporary_Parent_Node_To_Current_Node_Travel_Cost < Parent_Node_To_Current_Node_Travel_Cost)
			{
				new_found_node.Parent_Node_Object = object_node.Node_Object;
				new_found_node.Parent_Node_Index_In_List = object_node.Node_Index_In_List;
				Parent_Node_To_Current_Node_Travel_Cost = Temporary_Parent_Node_To_Current_Node_Travel_Cost;
			}
		}

		////RRT* Algorithm
		////Scanning for Neighbors in a Search Radius
		//List<Struct_Node> Neighbors_In_Radius = new();
		//{
		//	for (int index = 0; index < Mapped_Nodes_Graph.Count(); index++)
		//	{
		//		Struct_Node neighbor = Mapped_Nodes_Graph[index];
		//		int Travel_Cost = (int)(neighbor.Node_Object.transform.position - new_found_node.Node_Object.transform.position).sqrMagnitude;
		//		if (Travel_Cost < Neighbor_Updation_Search_Radius_In_m)
		//			Neighbors_In_Radius.Add(neighbor);
		//	}
		//}

		//{
		//	//Search for the Neighbor Node which has the shortest distance from start
		//	int Start_To_Current_Node_Travel_Cost = int.MaxValue;
		//	Struct_Node Parent_Node = default;
		//	for (int index = 0; index < Neighbors_In_Radius.Count(); index++)
		//	{
		//		Struct_Node neighbor = Neighbors_In_Radius[index];
		//		int Temporary_Parent_Node_To_Current_Node_Travel_Cost = (int)((neighbor.Node_Object.transform.position - new_found_node.Node_Object.transform.position).sqrMagnitude);
		//		int Temporary_Start_To_Current_Node_Travel_Cost = neighbor.Start_To_Current_Node_Travel_Cost + Temporary_Parent_Node_To_Current_Node_Travel_Cost;
		//		if (Temporary_Start_To_Current_Node_Travel_Cost < Start_To_Current_Node_Travel_Cost)
		//		{
		//			Start_To_Current_Node_Travel_Cost = Temporary_Start_To_Current_Node_Travel_Cost;
		//			Parent_Node = neighbor;
		//		}
		//	}
		//	new_found_node.Parent_Node_Object = Parent_Node.Node_Object;
		//	new_found_node.Parent_Node_Index_In_List = Parent_Node.Node_Index_In_List;
		//	new_found_node.Parent_Node_To_Current_Node_Travel_Cost = (int)((Parent_Node.Node_Object.transform.position - new_found_node.Node_Object.transform.position).sqrMagnitude);
		//	new_found_node.Start_To_Current_Node_Travel_Cost = Start_To_Current_Node_Travel_Cost;
		//	Neighbors_In_Radius.Remove(Parent_Node);
		//}

		//{
		//	//Update the Neighbor Nodes with the Current Node if the current node is having better cost than the earlier node they have as parent
		//	for (int index = 0; index < Neighbors_In_Radius.Count(); index++)
		//	{
		//		Struct_Node neighbor = Neighbors_In_Radius[index];
		//		int Old_Start_To_Node_TraveL_Cost = neighbor.Start_To_Current_Node_Travel_Cost;
		//		int New_Found_Node_To_Neighbor_Travel_Cost = (int)((neighbor.Node_Object.transform.position - new_found_node.Node_Object.transform.position).sqrMagnitude);
		//		int New_Start_To_Current_Node_Travel_Cost = new_found_node.Start_To_Current_Node_Travel_Cost + New_Found_Node_To_Neighbor_Travel_Cost;
		//		if (New_Start_To_Current_Node_Travel_Cost < Old_Start_To_Node_TraveL_Cost)
		//		{
		//			neighbor.Parent_Node_Object = new_found_node.Node_Object;
		//			neighbor.Parent_Node_Index_In_List = new_found_node.Node_Index_In_List;
		//			neighbor.Parent_Node_To_Current_Node_Travel_Cost = New_Found_Node_To_Neighbor_Travel_Cost;
		//			neighbor.Start_To_Current_Node_Travel_Cost = New_Start_To_Current_Node_Travel_Cost;
		//			neighbor.Line.SetPosition(0, new_found_node.Node_Object.transform.position);
		//			neighbor.Line.SetPosition(1, neighbor.Node_Object.transform.position);
		//		}
		//	}
		//}

		new_found_node.Line = Instantiate(Line_Renderer, this.transform);
			new_found_node.Line.SetPosition(0, new_found_node.Node_Object.transform.position);
			new_found_node.Line.SetPosition(1, new_found_node.Parent_Node_Object.transform.position);
			this.Mapped_Nodes_Graph.Add(new_found_node);
	}

	bool Retrace_Path ()
	{
		int Shortest_Path = int.MaxValue;
		Struct_Node closest_node = new();
		closest_node.End_Goal_To_Current_Node_Travel_Cost = int.MaxValue;
		for (int i = 0; i < Mapped_Nodes_Graph.Count(); i++)
		{
			Struct_Node node = Mapped_Nodes_Graph[i];
			if (node.End_Goal_To_Current_Node_Travel_Cost < Shortest_Path)
				Shortest_Path = node.End_Goal_To_Current_Node_Travel_Cost;
			{
				if (node.End_Goal_To_Current_Node_Travel_Cost < Acceptable_Error_Distance_In_m)
					closest_node = Mapped_Nodes_Graph[i];
			}
		}

		if (closest_node.End_Goal_To_Current_Node_Travel_Cost != int.MaxValue)
		{
			Debug.Log("Retracing Points");
			Generated_Path.Add(closest_node.Node_Object.transform.position);
			while (closest_node.Parent_Node_Object != null)
			{
				closest_node.Node_Object.GetComponent<Renderer>().material.SetColor("_Color", Complete_Search_Color);
				Generated_Path.Add(closest_node.Parent_Node_Object.transform.position);
				closest_node = Mapped_Nodes_Graph[closest_node.Parent_Node_Index_In_List];
			}
			Generated_Path.Reverse();
			return true;
		}
		return false;
	}

	void Plot_Path ()
	{
		Debug.Log("Plotting Paths");
		Vector3[] points = Generated_Path.ToArray();
		LineRenderer line = gameObject.AddComponent<LineRenderer>();
		line.widthMultiplier = 0.3f;
		line.positionCount = points.Count();
		line.GetComponent<Renderer>().material.SetColor("_Color", Complete_Search_Color);
		line.SetPositions(points);
	}
	#endregion //CustomMethods
}
