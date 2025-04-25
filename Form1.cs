// Form1.cs
using System;
using System.Collections.Generic;
using DijkstraWinForms;
using System.Windows.Forms;

namespace DijkstraWinForms
{
    public partial class Form1 : Form
    {
        Dictionary<string, Vertex> graph = new Dictionary<string, Vertex>();

        public Form1()
        {
            InitializeComponent();
        }

        private void btnInit_Click(object sender, EventArgs e)
        {
            Vertex A = new Vertex("A");
            Vertex B = new Vertex("B");
            Vertex C = new Vertex("C");
            Vertex D = new Vertex("D");

            A.AddEdge(1, B);
            A.AddEdge(4, C);
            B.AddEdge(2, C);
            B.AddEdge(6, D);
            C.AddEdge(3, D);

            graph.Clear();
            graph["A"] = A;
            graph["B"] = B;
            graph["C"] = C;
            graph["D"] = D;

            cmbStart.Items.Clear();
            cmbEnd.Items.Clear();

            foreach (var name in graph.Keys)
            {
                cmbStart.Items.Add(name);
                cmbEnd.Items.Add(name);
            }

            MessageBox.Show("Đã khởi tạo đồ thị mẫu.");
        }

        private void btnRun_Click(object sender, EventArgs e)
        {
            if (cmbStart.SelectedItem == null || cmbEnd.SelectedItem == null)
            {
                MessageBox.Show("Vui lòng chọn đỉnh bắt đầu và kết thúc.");
                return;
            }

            string start = cmbStart.SelectedItem.ToString();
            string end = cmbEnd.SelectedItem.ToString();

            foreach (var vertex in graph.Values)
            {
                vertex.MinDistance = double.MaxValue;
                vertex.Predecessor = null;
                vertex.wasVisited = false;
            }

            Dijkstra algo = new Dijkstra();
            algo.FindShortestPath(graph[start], graph[end]);

            lstResult.Items.Clear();
            lstResult.Items.Add("Đường đi ngắn nhất từ " + start + " đến " + end + ":");

            Vertex v = graph[end];
            Stack<string> path = new Stack<string>();
            while (v != null)
            {
                path.Push(v.Name);
                v = v.Predecessor;
            }

            lstResult.Items.Add("Tổng trọng số: " + graph[end].MinDistance);
            lstResult.Items.Add("Đường đi: " + string.Join(" -> ", path));
        }

        private void cmbStart_SelectedIndexChanged(object sender, EventArgs e)
        {
            
        }

        private void lstResult_SelectedIndexChanged(object sender, EventArgs e)
        {

        }
    }

    public class Vertex
    {
        public bool wasVisited { get; set; } = false;
        public string Name { get; set; }
        public Vertex Predecessor { get; set; } = null;
        public List<Edge> Neighbors { get; set; } = new List<Edge>();
        public double MinDistance { get; set; } = double.MaxValue;

        public Vertex(string name)
        {
            Name = name;
        }

        public double CompareTo(Vertex other)
        {
            return this.MinDistance < other.MinDistance ? -1 : this.MinDistance > other.MinDistance ? 1 : 0;
        }

        public void AddEdge(double weight, Vertex destinationVertex)
        {
            Edge edge = new Edge(this, destinationVertex, weight);
            Neighbors.Add(edge);
        }
    }

    public class Edge
    {
        public Vertex From { get; set; }
        public Vertex To { get; set; }
        public double Weight { get; set; }

        public Edge(Vertex from, Vertex to, double weight)
        {
            From = from;
            To = to;
            Weight = weight;
        }
    }

    public class MinHeap
    {
        private List<Vertex> heap = new List<Vertex>();

        public void Push(Vertex node)
        {
            heap.Add(node);
            BubbleUp(heap.Count - 1);
        }

        public Vertex Pop()
        {
            if (heap.Count == 0)
                throw new InvalidOperationException("Heap is empty");

            Vertex minNode = heap[0];
            Vertex lastNode = heap[heap.Count - 1];
            heap[0] = lastNode;
            heap.RemoveAt(heap.Count - 1);
            BubbleDown(0);

            return minNode;
        }

        private void BubbleUp(int index)
        {
            int parentIndex = (index - 1) / 2;

            while (index > 0 && heap[index].MinDistance < heap[parentIndex].MinDistance)
            {
                Swap(index, parentIndex);
                index = parentIndex;
                parentIndex = (index - 1) / 2;
            }
        }

        private void BubbleDown(int index)
        {
            int left = 2 * index + 1;
            int right = 2 * index + 2;
            int smallest = index;

            if (left < heap.Count && heap[left].MinDistance < heap[smallest].MinDistance)
                smallest = left;

            if (right < heap.Count && heap[right].MinDistance < heap[smallest].MinDistance)
                smallest = right;

            if (smallest != index)
            {
                Swap(index, smallest);
                BubbleDown(smallest);
            }
        }

        private void Swap(int i, int j)
        {
            var temp = heap[i];
            heap[i] = heap[j];
            heap[j] = temp;
        }

        public int Count()
        {
            return heap.Count;
        }
    }

    public class Dijkstra
    {
        public void FindShortestPath(Vertex start, Vertex end)
        {
            MinHeap heap = new MinHeap();
            start.MinDistance = 0;
            heap.Push(start);

            while (heap.Count() > 0)
            {
                Vertex current = heap.Pop();
                if (current.wasVisited)
                    continue;
                current.wasVisited = true;

                if (current == end)
                    break;

                foreach (Edge edge in current.Neighbors)
                {
                    Vertex neighbor = edge.To;
                    if (!neighbor.wasVisited)
                    {
                        double newDist = current.MinDistance + edge.Weight;
                        if (newDist < neighbor.MinDistance)
                        {
                            neighbor.MinDistance = newDist;
                            neighbor.Predecessor = current;
                            heap.Push(neighbor);
                        }
                    }
                }
            }
        }
    }
}